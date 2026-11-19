"""01: read the PC service. No new robot connections and no motion commands."""
import json
from quest_e6_sdk import TeleopClient

if __name__ == "__main__":
    state = TeleopClient().status()
    group = state["teleop"]
    print(json.dumps({"mode": state["mode"],
                      "teleop": {key: group.get(key) for key in ("state", "reason", "armed", "participants")},
                      "profile_id": (state["configured_ranged_ik"] or {}).get("profile_id")},
                     ensure_ascii=False, indent=2))
