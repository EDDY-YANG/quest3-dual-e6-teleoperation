"""Explicit-source, bounded transport around the pinned official V4 formatter.

This adapter is NOT a safety controller. It permits explicit state/motion calls
in ALLOWED_COMMANDS but never enables, clears errors, starts loops, reconnects
or replays a command automatically. Callers own motion authorization and limits.
An acknowledged command is not proof of completed motion or a stopped robot.
"""

import importlib.util
import ipaddress
import math
import re
import socket
import threading
import time
from pathlib import Path

UPSTREAM_COMMIT = "82bce656c6e1a1c71599901baa93ed45bb50963d"
STOP_REPLY_TIMEOUT_S = 10.0
ENABLE_REPLY_TIMEOUT_S = 10.0
_SDK_PATH = Path(__file__).resolve().parent / "vendor/dobot-official/dobot_api.py"
if not _SDK_PATH.is_file():
    raise ImportError(f"Pinned official SDK missing: {_SDK_PATH}")
_spec = importlib.util.spec_from_file_location("_dobot_official_main", _SDK_PATH)
_official = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_official)

ALLOWED_COMMANDS = frozenset(
    {
        "RobotMode",
        "GetAngle",
        "GetPose",
        "GetErrorID",
        "GetCurrentCommandID",
        "InverseKin",
        "PositiveKin",
        "Stop",
        "EnableRobot",
        "DisableRobot",
        "SpeedFactor",
        "ServoJ",
        "MovJ",
    }
)
_PARAMETER_NAMES = frozenset(
    {"user", "tool", "useJointNear", "JointNear", "t", "aheadtime", "gain", "joint", "a", "v", "cp"}
)
_NOT_TCP = "Control Mode Is Not Tcp"
_MAX_REPLY = 16384


class DobotCommandRejected(ValueError):
    """Rejected before transmission: outside this adapter's permitted API."""


class DobotReplyError(ValueError):
    """Malformed or inconsistent reply; do not infer that the command failed to run."""


class DobotTransportError(ConnectionError):
    """Connection failed/closed; a transmitted command may have been accepted.

    The adapter closes the socket and does not retry. Explicit operator recovery
    and a new adapter instance are required; this exception is not a stop ACK.
    """


class DobotCommandError(RuntimeError):
    def __init__(self, reply):
        self.reply = reply
        super().__init__(f"DOBOT command not successful: {reply['raw']!r}")


def _split_reply(raw):
    if not isinstance(raw, str):
        raise DobotReplyError("Reply must be a string")
    text = raw.strip()
    if text == _NOT_TCP:
        return None, None, None
    prefix = re.match(r"^(-?\d+)\s*,\s*\{", text)
    if not prefix:
        raise DobotReplyError("Missing ErrorID and payload envelope")
    start = prefix.end()
    depth = 1
    end = None
    for index in range(start, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                end = index
                break
    if end is None:
        raise DobotReplyError("Unclosed payload envelope")
    suffix = re.fullmatch(r",\s*([A-Za-z][A-Za-z0-9]*)\([^();\r\n]*\);", text[end + 1 :])
    if not suffix:
        raise DobotReplyError("Invalid command echo or multiple replies")
    return int(prefix[1]), text[start:end], suffix[1]


def parse_reply(raw):
    """Return ErrorID, raw payload body (without outer braces), and original text.

    E.g. ``0,{1,2},GetAngle();`` -> payload ``"1,2"``. Nested alarm payloads
    remain strings; callers must parse the specific command's payload strictly.
    The known non-TCP-mode message returns error_id=None and payload=None.
    """
    error_id, payload, _ = _split_reply(raw)
    return {"error_id": error_id, "payload": payload, "raw": raw}


def require_success(raw):
    """Parse and require ErrorID == 0; never clear an error or retry."""
    reply = parse_reply(raw)
    if reply["error_id"] != 0:
        raise DobotCommandError(reply)
    return reply


def _validate_command(command):
    if not isinstance(command, str) or len(command) > 4096:
        raise DobotCommandRejected("Invalid command string")
    match = re.fullmatch(r"([A-Za-z][A-Za-z0-9]*)\(([A-Za-z0-9_+.,{}= \-]*)\)", command)
    if not match or match[1] not in ALLOWED_COMMANDS:
        raise DobotCommandRejected("Command not in the explicit allowlist")
    # The supported commands have numeric arguments and named numeric options.
    # No string/file/IO parameters or nested command syntax are accepted.
    body = match[2]
    numeric_pattern = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
    if any(not math.isfinite(float(value)) for value in re.findall(numeric_pattern, body)):
        raise DobotCommandRejected("Non-finite numeric argument")
    stripped_numbers = re.sub(numeric_pattern, "", body)
    names = set(re.findall(r"[A-Za-z_][A-Za-z_0-9]*", stripped_numbers))
    if not names <= _PARAMETER_NAMES:
        raise DobotCommandRejected("Unknown parameter, NaN or infinity in command")
    depth = 0
    for char in body:
        if char == "{":
            depth += 1
        if char == "}":
            depth -= 1
        if depth < 0:
            raise DobotCommandRejected("Unbalanced argument braces")
    if depth:
        raise DobotCommandRejected("Unbalanced argument braces")
    if match[1] == "MovJ":
        # Narrow return-to-start interface: joint target, explicit bounded
        # speed/acceleration, zero blending. Cartesian/IO/file variants stay blocked.
        joint_move = re.fullmatch(r"joint=\{([^{}]+)\},a=(\d+),v=(\d+),cp=0", body)
        if not joint_move:
            raise DobotCommandRejected("MovJ requires explicit joint/a/v/cp=0 return format")
        values = joint_move[1].split(",")
        if len(values) != 6 or any(not re.fullmatch(numeric_pattern, x) for x in values):
            raise DobotCommandRejected("MovJ requires six finite joints")
        if not all(1 <= int(x) <= 100 for x in joint_move.groups()[1:]):
            raise DobotCommandRejected("MovJ speed/acceleration ratio outside 1..100")
    return match[1]


class BoundDobotDashboard(_official.DobotApiDashboard):
    """Official command formatting with fail-closed, source-bound TCP transport.

    Construction opens exactly one connection and sends zero robot commands.
    The default 0.4-second budget covers ordinary transactions. Stop gets a
    separate 10-second reply budget, as does motor enabling. Transmission is immediate.
    Neither budget is a validated physical stop-time bound.
    ``close()`` only closes the socket; it does not stop/disable the robot.
    """

    def __init__(self, robot_ip, local_source_ip, *, port=29999, timeout_s=0.4):
        # Do not call the upstream constructor: it opens an unbound connection.
        self.socket_dobot = None
        self._transport_lock = threading.Lock()
        self.text_log = None
        self.ip, self.port, self.local_source_ip = robot_ip, port, local_source_ip
        if type(timeout_s) not in (int, float) or not 0 < timeout_s <= 5:
            raise ValueError("timeout_s must be finite and in (0, 5]")
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("Invalid TCP port")
        for address in (robot_ip, local_source_ip):
            if not isinstance(address, str):
                raise ValueError("Explicit robot and source IPv4 addresses are required")
            ip = ipaddress.IPv4Address(address)
            if ip.is_unspecified or ip.is_multicast:
                raise ValueError("Unspecified/multicast addresses are not accepted")
        self.timeout_s = float(timeout_s)
        try:
            self.socket_dobot = socket.create_connection(
                (self.ip, self.port),
                timeout=self.timeout_s,
                source_address=(self.local_source_ip, 0),
            )
            self.socket_dobot.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.socket_dobot.settimeout(self.timeout_s)
            if self.socket_dobot.getsockname()[0] != self.local_source_ip:
                raise OSError("Socket did not bind the requested local source address")
        except OSError as exc:
            self.close()
            raise DobotTransportError(
                "Explicit-source connection failed; no fallback attempted"
            ) from exc

    @property
    def connected(self):
        """Local socket availability only, not a robot-mode or heartbeat check."""
        return self.socket_dobot is not None

    @property
    def local_address(self):
        return self.socket_dobot.getsockname() if self.socket_dobot is not None else None

    def sendRecvMsg(self, command):
        command_name = _validate_command(command)
        if command_name == "InverseKin":
            # Pinned upstream main spells this wire parameter incorrectly (issue #10).
            # Keep the Python API compatible, but use the documented controller spelling.
            command = command.replace(",JointNear=", ",jointNear=")
        return self._transaction(command, command_name)

    def RecoveryClearError(self):
        """One explicit recovery transaction; normal ClearError remains blocked."""
        return self._transaction("ClearError()", "ClearError")

    def _transaction(self, command, command_name):
        special_budget = {"Stop": STOP_REPLY_TIMEOUT_S, "EnableRobot": ENABLE_REPLY_TIMEOUT_S, "ClearError": 10.0}
        timeout_s = max(self.timeout_s, special_budget.get(command_name, self.timeout_s))
        deadline = time.monotonic() + timeout_s
        if not self._transport_lock.acquire(timeout=timeout_s):
            self.close()
            raise DobotTransportError("Transport lock timed out; connection closed, no retry")
        send_attempted = False
        try:
            sock = self.socket_dobot
            if sock is None:
                raise DobotTransportError("Connection is closed; explicit recovery is required")

            def remaining():
                budget = deadline - time.monotonic()
                if budget <= 0:
                    raise TimeoutError("Command deadline exceeded")
                sock.settimeout(budget)

            remaining()
            send_attempted = True  # A failed sendall may have transmitted a partial command.
            sock.sendall(command.encode("ascii"))
            raw = b""
            while True:
                remaining()
                chunk = sock.recv(1024)
                if not chunk:
                    raise EOFError("Connection closed before a complete reply")
                raw += chunk
                if len(raw) > _MAX_REPLY:
                    raise DobotReplyError("Oversized reply")
                if b";" in raw or raw.strip() == _NOT_TCP.encode("ascii"):
                    text = raw.decode("utf-8", "strict")
                    _, _, echo = _split_reply(text)
                    if echo is not None and echo != command_name:
                        raise DobotReplyError("Reply echo does not match the in-flight command")
                    return text
        except (OSError, EOFError, ValueError) as exc:
            self.close()
            outcome = "outcome may be unknown" if send_attempted else "was not sent"
            raise DobotTransportError(
                f"{command_name}: {outcome} ({type(exc).__name__}: {exc}); "
                "connection closed without reconnect or replay"
            ) from exc
        finally:
            self._transport_lock.release()

    def RecoverStop(self):
        """Explicit operator recovery only: one new connection, one Stop, then close.

        The original command connection stays closed. No movement command is
        replayed, and a fresh preparation is required before later movement.
        """
        if self.connected:
            raise DobotCommandRejected("Use Stop on the existing connection")
        if not self._transport_lock.acquire(timeout=self.timeout_s):
            raise DobotTransportError("Recovery refused while a transaction is in flight")
        try:
            if self.connected:
                raise DobotCommandRejected("Recovery requires a closed original connection")
            with BoundDobotDashboard(
                self.ip, self.local_source_ip, port=self.port, timeout_s=2.0
            ) as recovery:
                return recovery.Stop()
        finally:
            self._transport_lock.release()

    def close(self):
        sock = getattr(self, "socket_dobot", None)
        self.socket_dobot = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __del__(self):
        self.close()


def _blocked_method(name):
    def reject(self, *args, **kwargs):
        raise DobotCommandRejected(f"{name} is outside the supported adapter API")

    reject.__name__ = name
    return reject


# Some upstream methods (notably GetError) make HTTP requests directly instead
# of going through sendRecvMsg. Explicitly block those and the legacy transport
# methods as well as all non-allowlisted public robot methods.
for _name in dir(_official.DobotApiDashboard):
    if _name.startswith("_") or _name in ALLOWED_COMMANDS or _name in {"sendRecvMsg", "close"}:
        continue
    if callable(getattr(_official.DobotApiDashboard, _name)):
        setattr(BoundDobotDashboard, _name, _blocked_method(_name))
