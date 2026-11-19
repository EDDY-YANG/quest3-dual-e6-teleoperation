"""Run with python -m quest_e6_sdk. No command means help, not motion."""

import argparse
import json
import sys

from . import SDKError, TeleopClient, __version__
from .doctor import inspect


def main(argv=None):
    parser = argparse.ArgumentParser(description="Quest/E6 SDK: explicit operations through the existing PC backend")
    parser.add_argument("--version", action="version", version=__version__)
    commands = parser.add_subparsers(dest="command")
    doctor = commands.add_parser("doctor", help="Offline files/dependencies; no network")
    doctor.add_argument("--json", action="store_true")
    for name, help_text in {
        "status": "Read cached full state", "quest": "Read real/stale controller input",
        "monitor": "Read per-arm feedback", "headset": "Read HUD capabilities/calibration",
        "lan-info": "Read current LAN URL and pair code", "lan-enable": "Start prepared LAN listener",
        "lan-renew": "Replace pairing code/cookie", "connect-usb": "Set up authorized USB Quest",
        "stop": "Request Stop; inspect feedback confirmation",
        "capture-diagnostic": "Write short read-only diagnostic recording",
    }.items():
        commands.add_parser(name, help=help_text)
    for name in ("connect-feedback", "disconnect-feedback", "check-robot", "prepare", "arm"):
        child = commands.add_parser(name, help="Permits real following" if name == "arm" else None)
        auto = name in ("prepare", "arm")
        child.add_argument("--side", choices=("left", "right", "both", "auto") if auto
                           else ("left", "right", "both"), default="auto" if auto else "both")
    args = parser.parse_args(argv)
    if args.command is None:
        parser.print_help()
        return 0
    try:
        if args.command == "doctor":
            result = inspect()
            if not args.json:
                for row in result["checks"]:
                    print(f"{row['status']:4}  {row['name']}: {row['detail']}")
                print("Scope: offline only; no physical readiness or motion acceptance is implied.")
                return 0 if result["ok"] else 1
        else:
            method = {"headset": "headset_status", "lan-enable": "enable_lan",
                      "lan-renew": "renew_pairing"}.get(args.command, args.command.replace("-", "_"))
            client = TeleopClient()
            result = getattr(client, method)(**({"side": args.side} if hasattr(args, "side") else {}))
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        if args.command == "doctor":
            return 0 if result["ok"] else 1
        if args.command == "stop" and (result.get("armed") or result.get("state") == "FAULT"
                                      or not (result.get("stop_evidence") or {}).get("feedback_confirmed")):
            return 1
        return 0
    except (SDKError, ValueError) as exc:
        print(json.dumps({"error": str(exc), "outcome_unknown": getattr(exc, "outcome_unknown", False),
                          "automatic_retry": False}, ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
