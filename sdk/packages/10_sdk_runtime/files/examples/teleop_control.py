"""05: explicit Arm or Stop. Arm allows REAL motion from subsequent live Grip."""
import argparse
import json
from quest_e6_sdk import TeleopClient

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("arm", "stop"))
    parser.add_argument("--side", choices=("left", "right", "both", "auto"), default="auto")
    args = parser.parse_args()
    client = TeleopClient()
    result = client.arm(args.side) if args.action == "arm" else client.stop()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.action == "stop" and not (result.get("stop_evidence") or {}).get("feedback_confirmed"):
        raise SystemExit("Stop has NOT been confirmed. Read state/reason and per-arm evidence.")
