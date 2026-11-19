"""04: explicit robot queries/model preparation. Does not Arm or issue motion."""
import argparse
import json
from quest_e6_sdk import TeleopClient

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right", "both", "auto"), required=True)
    args = parser.parse_args()
    print(json.dumps(TeleopClient().prepare(args.side), ensure_ascii=False, indent=2))
