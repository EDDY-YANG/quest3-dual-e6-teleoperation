"""Local, opt-in device addresses. Reading and validation perform no network I/O."""
import argparse
import ipaddress
import json
from pathlib import Path

SIDES = ("left", "right")
KEYS = {f"{side}_e6{suffix}_ip" for side in SIDES for suffix in ("", "_local")}
LOCAL_FILE = Path("runtime") / "devices.local.json"
PRIVATE_NETS = tuple(ipaddress.ip_network(n) for n in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16"))


def validate(config):
    if not isinstance(config, dict) or set(config) != KEYS:
        raise ValueError("Device configuration must contain the four fields in config.example.json")
    sides = []
    hosts, robots = set(), set()
    for side in SIDES:
        robot, host = (config[f"{side}_e6{suffix}_ip"] for suffix in ("", "_local"))
        if robot is None and host is None:
            continue
        for value in (robot, host):
            if not isinstance(value, str):
                raise ValueError(f"Set both addresses for {side}, or leave both null")
            address = ipaddress.IPv4Address(value)
            if not any(address in net for net in PRIVATE_NETS):
                raise ValueError(f"Use the private IPv4 addresses for the {side} robot and its PC adapter")
        sides.append(side)
        hosts.add(host)
        robots.add(robot)
    if hosts & robots:
        raise ValueError("A PC source address must differ from both robot addresses")
    if len(sides) == 2 and len(robots) == 1 and len(hosts) != 2:
        raise ValueError("Same-address robots need separate links and distinct PC source addresses")
    return tuple(sides)


def load_device_config(root):
    path = Path(root) / LOCAL_FILE
    if not path.exists():
        return dict.fromkeys(sorted(KEYS))
    config = json.loads(path.read_text(encoding="utf-8-sig"))
    validate(config)
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument("--init", action="store_true", help="Create an empty local template once")
    actions.add_argument("--check", action="store_true", help="Check addresses without connecting")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    path = root / LOCAL_FILE
    if args.init:
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with path.open("x", encoding="utf-8") as stream:
                stream.write((root / "config.example.json").read_text(encoding="utf-8"))
            print(f"Edit {path} with your own network settings.")
        except FileExistsError:
            print(f"Your existing settings were kept: {path}")
    else:
        sides = validate(load_device_config(root))
        if not sides:
            parser.exit(1, "No robot configured. Run --init, then edit runtime/devices.local.json.\n")
        print("Configured sides: " + ", ".join(sides) + ". File check only.")


if __name__ == "__main__":
    main()
