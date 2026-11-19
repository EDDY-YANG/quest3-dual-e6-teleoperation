"""Read-only PC/Quest/E6 communication measurements; never sends robot commands.

Only GET /api/status on loopback and GET /protocol/exchange on the configured
E6 HTTP status port are used. Each E6 socket binds the configured source IP;
there is no unbound fallback, SDK import, control socket, retry or disk stream.
HTTP round trips include a fresh TCP connection and controller/server processing.
They do not measure WebSocket, ServoJ or physical end-to-end response time.
"""

import argparse
import ipaddress
import json
import math
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
from http.client import HTTPConnection, HTTPException
from pathlib import Path
from statistics import median

ROOT = Path(__file__).resolve().parent
CST = timezone(timedelta(hours=8))


def stamp():
    return datetime.now(CST).isoformat(timespec="milliseconds")


def private_ip(value):
    address = ipaddress.IPv4Address(value)
    if not address.is_private or address.is_loopback or address.is_unspecified:
        raise ValueError("A private, non-loopback IPv4 address is required")
    return str(address)


def read_json(host, port, path, source=None):
    connection = HTTPConnection(
        host, port, timeout=2, source_address=(source, 0) if source else None
    )
    started = time.perf_counter()
    try:
        connection.request("GET", path, headers={"Cache-Control": "no-cache"})
        response = connection.getresponse()
        body = response.read(262145)
        if len(body) > 262144:
            raise ValueError("Status response exceeded 256 KiB")
        if response.status != 200:
            raise ValueError(f"HTTP {response.status}")
        data = json.loads(body)
        if not isinstance(data, dict):
            raise ValueError("Expected a JSON status object")
        return {
            "ok": True,
            "http_rtt_ms": round((time.perf_counter() - started) * 1000, 3),
            "data": data,
        }
    except (OSError, ValueError, HTTPException) as exc:
        return {"ok": False, "http_rtt_ms": None, "error": str(exc)}
    finally:
        connection.close()


def number(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def snapshot(pool, robot_ip, local_ip):
    pc_task = pool.submit(read_json, "127.0.0.1", 8765, "/api/status")
    e6_task = pool.submit(read_json, robot_ip, 22000, "/protocol/exchange", local_ip)
    pc, e6 = pc_task.result(), e6_task.result()
    status, exchange = pc.pop("data", {}), e6.pop("data", {})
    quest = status.get("quest") or {}
    frame = quest.get("frame") or {}
    network = (frame.get("diagnostics") or {}).get("network") or {}
    rtt, age = network.get("http_rtt_ms"), network.get("http_rtt_age_ms")
    fresh = quest.get("connected") is True and number(quest.get("age_ms")) and quest["age_ms"] < 500
    valid_rtt = fresh and number(rtt) and number(age) and age <= 5000
    trial = status.get("left_trial") or {}
    return {
        "checked_at": stamp(),
        "pc": pc,
        "e6": {
            **e6,
            "robot_ip": robot_ip,
            "source_ip": local_ip,
            **{
                key: exchange.get(key)
                for key in (
                    "powerState",
                    "remoteControl",
                    "controlMode",
                    "prjState",
                    "emergencyStop",
                    "isMotion",
                )
            },
        },
        "quest": {
            **{key: quest.get(key) for key in ("status", "connected", "age_ms", "receive_hz")},
            "http_rtt_ms": rtt if valid_rtt else None,
            "rtt_measurement_age_ms": age if valid_rtt else None,
            "rtt_status": "CLIENT_REPORTED_HTTP_RTT"
            if valid_rtt
            else "NO_FRESH_CLIENT_MEASUREMENT",
            "reconnects": network.get("reconnects") if fresh else None,
            "buffered_bytes": network.get("buffered_bytes") if fresh else None,
            "skipped_frames": network.get("skipped_frames") if fresh else None,
        },
        "left_trial": {
            key: trial.get(key)
            for key in ("state", "armed", "reason", "feedback", "command_timings", "stop_evidence")
        },
        "scope": "HTTP_STATUS_ONLY_NOT_MOTION_READINESS_OR_END_TO_END_LATENCY",
    }


def stats(values):
    ordered = sorted(value for value in values if number(value))
    if not ordered:
        return {"n": 0, "median_ms": None, "p95_ms": None, "max_ms": None}
    return {
        "n": len(ordered),
        "min_ms": ordered[0],
        "median_ms": round(median(ordered), 3),
        "p95_ms": ordered[math.ceil(len(ordered) * 0.95) - 1],
        "max_ms": ordered[-1],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--watch", action="store_true", help="Monitor until Ctrl+C; RAM only")
    parser.add_argument("--samples", type=int, default=10, help="Finite sample count (default: 10)")
    parser.add_argument("--interval", type=float, default=2, help="Seconds between polls (min: 1)")
    parser.add_argument("--save", action="store_true", help="Save one final summary in runtime/")
    args = parser.parse_args()
    if args.samples < 1 or not math.isfinite(args.interval) or args.interval < 1:
        parser.error("samples must be positive and interval must be at least 1 second")
    from device_settings import load_device_config, validate
    config = load_device_config(ROOT)
    if "left" not in validate(config):
        raise ValueError("DEVICE_NOT_CONFIGURED: configure the left arm before probing")
    robot_ip = private_ip(config["left_e6_ip"])
    local_ip = private_ip(config["left_e6_local_ip"])
    if robot_ip == local_ip:
        parser.error("E6 and PC source addresses must differ")
    history, total, started = deque(maxlen=120), 0, stamp()
    print("READ ONLY: HTTP status GETs; no 29999/30004 connection; Ctrl+C exits.", flush=True)
    with ThreadPoolExecutor(max_workers=2) as pool:
        try:
            while args.watch or total < args.samples:
                before = time.monotonic()
                record = snapshot(pool, robot_ip, local_ip)
                history.append(record)
                total += 1
                print(json.dumps(record, ensure_ascii=True, separators=(",", ":")), flush=True)
                if args.watch or total < args.samples:
                    time.sleep(max(0, args.interval - (time.monotonic() - before)))
        except KeyboardInterrupt:
            pass
    summary = {
        "started_at": started,
        "finished_at": stamp(),
        "polls": total,
        "window_polls": len(history),
        "window_limit": 120,
        "latency_definition": "HTTP request/response including new TCP connection; "
        "not control-command or physical latency",
        "pc_http": stats(item["pc"]["http_rtt_ms"] for item in history),
        "e6_http": stats(item["e6"]["http_rtt_ms"] for item in history),
        "quest_http": stats(item["quest"]["http_rtt_ms"] for item in history),
        "quest_sample_note": "Latest client measurement per poll; values may repeat",
        "latest": history[-1] if history else None,
    }
    if args.save:
        folder = ROOT / "runtime"
        folder.mkdir(exist_ok=True)
        path = folder / (
            datetime.now(CST).strftime("%Y-%m-%d-%H-%M-%S") + "-communication-baseline.json"
        )
        with path.open("x", encoding="utf-8") as stream:
            json.dump(summary, stream, ensure_ascii=True, indent=2)
        print(f"SAVED {path}", flush=True)
    print("SUMMARY " + json.dumps(summary, ensure_ascii=True), flush=True)


if __name__ == "__main__":
    main()
