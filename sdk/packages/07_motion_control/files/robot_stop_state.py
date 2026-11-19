"""Stop-only feedback acceptance. Collision protection never grants motion."""
import math
import time


def stop_feedback(packet):
    age = time.monotonic()-packet.get("host_monotonic_s", -math.inf)
    if not packet.get("connected") or not 0 <= age <= .2:
        raise ValueError("STOP_FEEDBACK_STALE_OR_DISCONNECTED")
    if packet.get("valid") and packet.get("robot_mode") in (4, 5, 7):
        return packet
    protected = (packet.get("packet_valid") is True and packet.get("robot_mode") == 11
                 and packet.get("robot_type") == 150 and packet.get("error_status") == 0
                 and packet.get("user") == 0 and packet.get("tool") == 0)
    joints = packet.get("joint_position_deg")
    if not protected or not isinstance(joints, list) or len(joints) != 6 or any(
        type(v) not in (int, float) or not math.isfinite(v) for v in joints
    ):
        raise ValueError("STOP_FEEDBACK_NOT_USABLE")
    # Preserve valid=False: these samples can prove stationary joint feedback
    # after Stop, but cannot be used by Arm, IK or a motion send guard.
    return packet
