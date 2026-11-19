"""02: sample real received Quest input at 2 Hz for ten seconds; no file logging."""
import json
import time
from quest_e6_sdk import TeleopClient

if __name__ == "__main__":
    client = TeleopClient(timeout_s=3)
    for _ in range(20):
        quest = client.quest()
        frame = quest.get("frame") or {}
        print(json.dumps({"status": quest.get("status"), "age_ms": quest.get("age_ms"),
                          "valid": quest.get("valid"), "hands": frame.get("hands")}, ensure_ascii=False))
        time.sleep(.5)
