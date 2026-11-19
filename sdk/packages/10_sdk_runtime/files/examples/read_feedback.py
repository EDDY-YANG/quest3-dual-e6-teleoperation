"""03: read cached monitor feedback. Connect it separately with connect-feedback."""
import json
from quest_e6_sdk import TeleopClient

if __name__ == "__main__":
    print(json.dumps(TeleopClient().monitor(), ensure_ascii=False, indent=2))
