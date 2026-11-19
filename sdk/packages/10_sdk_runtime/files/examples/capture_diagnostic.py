"""06: request one short real read-only capture; backend returns its save status."""
import json
from quest_e6_sdk import TeleopClient

if __name__ == "__main__":
    print(json.dumps(TeleopClient().capture_diagnostic(), ensure_ascii=False, indent=2))
