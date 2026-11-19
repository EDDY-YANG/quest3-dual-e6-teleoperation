"""Offline environment/asset checks. No sockets, robot import or writes."""

import hashlib
import importlib.metadata
import importlib.util
import json
import platform
import struct
import sys
from pathlib import Path
from device_settings import load_device_config, validate as validate_device_config

ROOT = Path(__file__).resolve().parents[1]


def inspect(root=ROOT):
    root = Path(root).resolve()
    checks = []

    def check(name, ok, detail, *, optional=False):
        checks.append({"name": name, "status": "PASS" if ok else "WARN" if optional else "FAIL",
                       "detail": detail})

    check("python", sys.version_info >= (3, 10), sys.version.split()[0])
    check("native_host", sys.platform == "win32" and struct.calcsize("P") == 8,
          f"{platform.system()} {platform.machine()}; bundled DLL requires Windows x64")
    for name, optional in (("aiohttp", False), ("numpy", False), ("requests", False),
                           ("cryptography", True)):
        found = importlib.util.find_spec(name) is not None
        try:
            version = importlib.metadata.version(name) if found else "missing"
        except importlib.metadata.PackageNotFoundError:
            version = "metadata unavailable"
        check("dependency:" + name, found, version, optional=optional)
    required = ("server.py", "teleop_group.py", "left_z_trial.py", "dobot_safe_sdk.py",
                "start_positions.py", "web/quest.js", "web/pc_monitor.html",
                "profiles/rangedik_world_anchor.json", "config.example.json", "device_settings.py")
    for name in required:
        check("file:" + name, (root / name).is_file(), name)
    try:
        configured = validate_device_config(load_device_config(root))
        check("device_binding", bool(configured),
              ("Address fields checked for " + ", ".join(configured)) if configured else
              "Unconfigured. For hardware use, run device_settings.py --init and enter your own addresses.",
              optional=True)
    except (OSError, ValueError, KeyError) as exc:
        check("config", False, str(exc))
    try:
        profile_path = root / "profiles/rangedik_world_anchor.json"
        profile = json.loads(profile_path.read_text(encoding="utf-8-sig"))
        library = (profile_path.parent / profile["native_library"]).resolve()
        manifest = json.loads(library.with_name("build-manifest.json").read_text(encoding="utf-8-sig"))
        check("native_sha256", hashlib.sha256(library.read_bytes()).hexdigest() == manifest["dll_sha256"],
              profile["profile_id"] + "; hash check only, not a DLL load/solve test")
        check("native_upstream", manifest["source"]["commit"] == "1c48d2ae408b4e024ee037641aac1e728267984e",
              manifest["source"]["commit"])
    except (OSError, ValueError, KeyError) as exc:
        check("native", False, str(exc))
    for manifest_name in ("vendor_manifest.json", "web/robot-assets/e6/manifest.json",
                          "web/robot-assets/three/manifest.json", "web/robot-assets/quest-touch-plus/manifest.json",
                          "SDK_MANIFEST.json"):
        manifest_path = root / manifest_name
        if manifest_name == "SDK_MANIFEST.json" and not manifest_path.exists():
            continue  # Present in exported snapshots, not the working tree.
        try:
            entries = json.loads(manifest_path.read_text(encoding="utf-8-sig"))["files"]
            if isinstance(entries, dict):
                pairs = [(root / name, digest) for name, digest in entries.items()]
            else:
                base = root if manifest_name == "SDK_MANIFEST.json" else manifest_path.parent
                pairs = [(base / entry.get("file", entry.get("path")), entry["sha256"]) for entry in entries]
            bad = [p.name for p, digest in pairs
                   if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != digest]
            check("integrity:" + manifest_name, not bad, f"{len(pairs)} files; mismatches: {bad}")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            check("integrity:" + manifest_name, False, str(exc))
    check("usb_adb", (root / "vendor/platform-tools/adb.exe").is_file(),
          "Optional for USB; LAN WebXR does not require ADB", optional=True)
    check("lan_certificate", (root / "runtime/lan-cert.pem").is_file(),
          "Presence only; regenerate for this PC's current WLAN IP before LAN setup", optional=True)
    return {"scope": "OFFLINE_FILES_AND_ENVIRONMENT_ONLY", "root": str(root),
            "ok": all(c["status"] != "FAIL" for c in checks), "checks": checks,
            "hardware_tested": False}
