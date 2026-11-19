"""Strict V4 feedback parsing and a source-bound, receive-only reader.

Layout: Dobot-Arm/TCP-IP-Python-V4, commit
55ec1ec82201aaf2e6d54aa47b6272bbaa14c815, protocol/feedback.py and official
V4.6.6 feedback table. Rx/Ry/Rz are fixed-axis XYZ rotations (Rz Ry Rx).
ActualQuaternion is wxyz on the wire; the public pose contract is xyzw.

Internal packet consistency is not a GetPose/firmware/frame acceptance test.
This module never grants motion permission and never writes to a robot socket.
"""

import asyncio
import copy
import ipaddress
import math
import struct
import time
from collections import deque
from contextlib import suppress

PACKET_SIZE = 1440
TEST_VALUE = 0x0123456789ABCDEF
FEEDBACK_PORT = 30004


def euler_xyz_deg_to_xyzw(angles):
    """Convert fixed-axis X/Y/Z degrees: R = Rz(rz) Ry(ry) Rx(rx)."""
    if len(angles) != 3 or any(not math.isfinite(x) for x in angles):
        raise ValueError("Invalid Euler angles")
    a, b, c = (math.radians(x) / 2 for x in angles)
    sa, ca, sb, cb, sc, cc = (
        math.sin(a),
        math.cos(a),
        math.sin(b),
        math.cos(b),
        math.sin(c),
        math.cos(c),
    )
    return [
        sa * cb * cc - ca * sb * sc,
        ca * sb * cc + sa * cb * sc,
        ca * cb * sc - sa * sb * cc,
        ca * cb * cc + sa * sb * sc,
    ]


def _six(data, offset):
    values = list(struct.unpack_from("<6d", data, offset))
    if any(not math.isfinite(x) for x in values):
        raise ValueError(f"Non-finite feedback field at offset {offset}")
    return values


def parse_packet(data, *, robot_id, host_monotonic_s, orientation_tolerance_deg=1.0):
    """Decode actual packet fields; malformed data raises ValueError.

    ``valid`` means a fresh caller-supplied receipt contains an internally
    consistent E6 pose in User 0 / Tool 0, with no controller error and mode
    4, 5 or 7. It does NOT mean motion is permitted. Mode 7 is valid telemetry
    but ``no_send_mapping_allowed`` remains false for the current idle mapper.
    The caller must separately compare GetPose/GetAngle before accepting the
    feedback frame contract for a real controller implementation.
    """
    if not isinstance(data, (bytes, bytearray, memoryview)) or len(data) != PACKET_SIZE:
        raise ValueError("Expected exactly 1440 feedback bytes")
    if struct.unpack_from("<H", data, 0)[0] != PACKET_SIZE:
        raise ValueError("Feedback length header mismatch")
    if struct.unpack_from("<Q", data, 48)[0] != TEST_VALUE:
        raise ValueError("Feedback marker mismatch; refusing to guess packet alignment")
    if not isinstance(robot_id, str) or not robot_id.strip():
        raise ValueError("Explicit source-binding robot_id is required")
    if type(host_monotonic_s) not in (int, float) or not math.isfinite(host_monotonic_s):
        raise ValueError("Finite PC monotonic receive timestamp is required")
    if not 0 < orientation_tolerance_deg <= 1:
        raise ValueError("Orientation tolerance must be in (0, 1] degrees")

    joint_position_deg = _six(data, 432)
    pose_mm_deg = _six(data, 624)
    user_value = _six(data, 1200)
    tool_value = _six(data, 1248)
    raw_wxyz = list(struct.unpack_from("<4d", data, 1384))
    if any(not math.isfinite(x) for x in raw_wxyz):
        raise ValueError("Non-finite ActualQuaternion")
    norm2 = sum(x * x for x in raw_wxyz)
    if abs(norm2 - 1) > 1e-3:
        raise ValueError("ActualQuaternion is not unit length; no placeholder substituted")
    # Preserve the raw wire values; normalize only validated floating-point roundoff.
    qw, qx, qy, qz = (x / math.sqrt(norm2) for x in raw_wxyz)
    quaternion = [qx, qy, qz, qw]
    from_euler = euler_xyz_deg_to_xyzw(pose_mm_deg[3:])
    dot = min(1.0, abs(sum(a * b for a, b in zip(quaternion, from_euler))))
    orientation_error = math.degrees(2 * math.acos(dot))
    if orientation_error > orientation_tolerance_deg:
        raise ValueError(f"Euler/quaternion mismatch: {orientation_error:.6f} degrees")

    mode = struct.unpack_from("<Q", data, 24)[0]
    user, tool, enable, error, robot_type = (data[i] for i in (1012, 1013, 1026, 1029, 1031))
    reasons = []
    if robot_type != 150:
        reasons.append("ROBOT_TYPE_NOT_E6")
    if user != 0 or tool != 0:
        reasons.append("NONZERO_USER_OR_TOOL_FRAME_NOT_ACCEPTED")
    if mode not in (4, 5, 7):
        reasons.append("ROBOT_MODE_NOT_USABLE_FOR_POSE_INPUT")
    if error != 0:
        reasons.append("CONTROLLER_ERROR_FLAG")
    if enable not in (0, 1) or (mode == 4 and enable != 0) or (mode in (5, 7) and enable != 1):
        reasons.append("ENABLE_FLAG_INCONSISTENT_WITH_MODE")
    return {
        "valid": not reasons,
        "invalid_reasons": reasons,
        "packet_valid": True,
        "execution_allowed": False,
        "requires_query_crosscheck": True,
        "no_send_mapping_allowed": not reasons and mode in (4, 5),
        "robot_id": robot_id,
        "frame_id": f"{robot_id}:user={user}:tool={tool}",
        "host_monotonic_s": float(host_monotonic_s),
        "controller_timestamp_raw": struct.unpack_from("<Q", data, 32)[0],
        "controller_runtime_ms": struct.unpack_from("<Q", data, 40)[0],
        "robot_mode": mode,
        "robot_type": robot_type,
        "enable_status": enable,
        "error_status": error,
        "user": user,
        "tool": tool,
        "user_value_mm_deg": user_value,
        "tool_value_mm_deg": tool_value,
        "position_mm": pose_mm_deg[:3],
        "quaternion_xyzw": quaternion,
        "quaternion_wxyz_raw": raw_wxyz,
        "pose_mm_deg": pose_mm_deg,
        "joint_position_deg": joint_position_deg,
        "euler_quaternion_error_deg": orientation_error,
        "scope": "V4_SELECTED_FIELDS_INTERNAL_CONSISTENCY_ONLY",
    }


class FeedbackReader:
    """Drain a real 30004 stream, keeping bounded in-memory state only.

    ``await start()`` starts the receive task. ``snapshot()`` returns at most
    the configured ordinary publication rate (10 Hz by default). Every packet
    is drained and checked; mode/error/frame changes publish immediately.
    ``await stop()`` cancels reception and clears usable pose state. Reconnects
    perform reads only and increment connection_generation, so callers can
    require explicit re-arming rather than resuming a previous clutch.
    """

    def __init__(
        self,
        ip,
        local_ip,
        robot_id,
        *,
        publish_hz=10.0,
        max_age_s=0.3,
        connect_timeout_s=2.0,
        read_timeout_s=0.5,
        reconnect_delay_s=1.0,
    ):
        for value in (ip, local_ip):
            address = ipaddress.ip_address(value)
            if (
                address.version != 4
                or not address.is_private
                or address.is_loopback
                or address.is_unspecified
            ):
                raise ValueError(
                    "Explicit private IPv4 robot and local interface addresses required"
                )
        if not isinstance(robot_id, str) or not robot_id.strip():
            raise ValueError("Explicit source-binding robot_id required")
        for value in (publish_hz, max_age_s, connect_timeout_s, read_timeout_s, reconnect_delay_s):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError("Feedback timing values must be positive and finite")
        if not 1 <= publish_hz <= 125:
            raise ValueError("publish_hz must be in [1, 125]")
        self.ip, self.local_ip, self.robot_id = ip, local_ip, robot_id
        self.publish_period = 1.0 / publish_hz
        self.max_age_s = max_age_s
        self.connect_timeout_s, self.read_timeout_s = connect_timeout_s, read_timeout_s
        self.reconnect_delay_s = reconnect_delay_s
        self.connected = False
        self.connection_generation = 0
        self.received_packets = 0
        self.last_error = "NOT_STARTED"
        self._latest = None
        self._receive_times = deque(maxlen=126)
        self._task = None
        self._stop = asyncio.Event()

    async def start(self):
        if self._task and not self._task.done():
            return
        self._stop.clear()
        self._invalidate("CONNECTING")
        self._task = asyncio.create_task(self._run(), name=f"feedback:{self.robot_id}")

    async def stop(self):
        self._stop.set()
        if self._task:
            self._task.cancel()
            with suppress(asyncio.CancelledError):
                await self._task
        self._task = None
        self._invalidate("STOPPED")

    def _invalidate(self, reason):
        self.connected = False
        self._latest = None
        self._receive_times.clear()
        self.last_error = reason

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        if type(now) not in (int, float) or not math.isfinite(now):
            raise ValueError("Finite monotonic time required")
        base = {
            "robot_id": self.robot_id,
            "ip": self.ip,
            "local_ip": self.local_ip,
            "connection_generation": self.connection_generation,
            "received_packets": self.received_packets,
            "connected": self.connected,
            "execution_allowed": False,
            "valid": False,
            "receive_hz": 0.0,
        }
        if not self.connected or self._latest is None:
            return {**base, "reason": self.last_error, "feedback_age_ms": None}
        age = now - self._latest["host_monotonic_s"]
        if not 0 <= age < self.max_age_s:
            return {**base, "reason": "STALE_OR_FUTURE_FEEDBACK", "feedback_age_ms": age * 1000}
        times = self._receive_times
        hz = (
            (len(times) - 1) / (times[-1] - times[0])
            if len(times) > 1 and times[-1] > times[0]
            else 0
        )
        return {
            **base,
            **copy.deepcopy(self._latest),
            "receive_hz": round(hz, 1),
            "feedback_age_ms": round(age * 1000, 3),
            "reason": "OK" if self._latest["valid"] else ";".join(self._latest["invalid_reasons"]),
        }

    async def _run(self):
        try:
            while not self._stop.is_set():
                writer = None
                try:
                    reader, writer = await asyncio.wait_for(
                        asyncio.open_connection(
                            self.ip,
                            FEEDBACK_PORT,
                            local_addr=(self.local_ip, 0),
                            limit=PACKET_SIZE * 4,
                        ),
                        self.connect_timeout_s,
                    )
                    actual_source = writer.get_extra_info("sockname")
                    if not actual_source or actual_source[0] != self.local_ip:
                        raise OSError("Socket source does not match the requested interface")
                    self.connection_generation += 1
                    self.connected = True
                    self.last_error = "WAITING_FOR_FEEDBACK"
                    last_controller_time = None
                    last_signature = None
                    last_publish = -math.inf
                    while not self._stop.is_set():
                        packet = await asyncio.wait_for(
                            reader.readexactly(PACKET_SIZE), self.read_timeout_s
                        )
                        received = time.monotonic()
                        decoded = parse_packet(
                            packet, robot_id=self.robot_id, host_monotonic_s=received
                        )
                        controller_time = decoded["controller_timestamp_raw"]
                        if (
                            last_controller_time is not None
                            and controller_time <= last_controller_time
                        ):
                            raise ValueError(
                                "Controller timestamp did not advance within this connection"
                            )
                        last_controller_time = controller_time
                        self.received_packets += 1
                        self._receive_times.append(received)
                        signature = tuple(
                            decoded[k]
                            for k in (
                                "robot_mode",
                                "enable_status",
                                "error_status",
                                "user",
                                "tool",
                                "valid",
                            )
                        )
                        if (
                            received - last_publish >= self.publish_period
                            or signature != last_signature
                        ):
                            self._latest = decoded
                            last_publish, last_signature = received, signature
                        self.last_error = "OK"
                except (
                    OSError,
                    asyncio.TimeoutError,
                    asyncio.IncompleteReadError,
                    ValueError,
                ) as exc:
                    self._invalidate(f"{type(exc).__name__}: {str(exc)[:300]}")
                finally:
                    if writer:
                        writer.close()
                        with suppress(OSError, asyncio.TimeoutError):
                            await asyncio.wait_for(writer.wait_closed(), 1.0)
                if not self._stop.is_set():
                    with suppress(asyncio.TimeoutError):
                        await asyncio.wait_for(self._stop.wait(), self.reconnect_delay_s)
        finally:
            self._invalidate("STOPPED" if self._stop.is_set() else "READER_ENDED")
