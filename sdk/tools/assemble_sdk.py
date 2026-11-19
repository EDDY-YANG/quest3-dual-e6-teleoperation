"""Assemble chain packages into an app without changing original module imports.

Only package source is editable. app/ is generated. Refuse to overwrite edits
made directly to generated files or to assemble while that app is running.
No environment setup, process launch, network request or robot I/O occurs.
"""
import argparse
import hashlib
import json
import zipfile
from datetime import datetime, timedelta, timezone
from pathlib import Path


def inside(root, name):
    path = (root / name).resolve()
    path.relative_to(root.resolve())
    return path


def digest(data):
    return hashlib.sha256(data).hexdigest()


def assemble(root, *, check=False):
    root = Path(root).resolve()
    app = root / "app"
    previous_path = root / "assembly-state.json"
    previous = json.loads(previous_path.read_text(encoding="utf-8")) if previous_path.exists() else {}
    entries = {}
    owners = {}
    for manifest in sorted((root / "packages").glob("*/package.json")):
        package = json.loads(manifest.read_text(encoding="utf-8"))
        for entry in package["files"]:
            target = entry["path"]
            inside(app, target)
            if target in entries:
                raise ValueError(f"Duplicate destination: {target}")
            source = inside(manifest.parent / "files", target)
            entries[target] = source.read_bytes()
            owners[target] = package["id"]
    if "server.py" not in entries or "sdk.ps1" not in entries:
        raise ValueError("Required packages are missing")
    current = {name: digest(data) for name, data in entries.items()}
    changed = [name for name, sha in current.items() if previous.get("files", {}).get(name) != sha]
    missing = [name for name in entries if not inside(app, name).is_file()]
    old_names = set(previous.get("files", {})) - set(entries)
    if old_names:
        raise ValueError(f"Package removals need a new export directory: {sorted(old_names)}")
    for name in entries:
        path = inside(app, name)
        if path.exists():
            actual = digest(path.read_bytes())
            expected = previous.get("files", {}).get(name)
            if actual != expected and actual != current[name]:
                raise ValueError(f"Generated app file was edited: {name}. Move the edit to packages/{owners[name]}/files/{name} first.")
    if check:
        if changed or missing:
            raise ValueError("Packages changed/unassembled. Stop this app, then run sdk.ps1 assemble.")
        return {"ok": True, "file_count": len(entries), "changed": 0}
    # A receipt may be stale. This conservative block is preferable to replacing
    # files underneath a process; normal shutdown removes it in the outer wrapper.
    if (app / "runtime/server-process.json").exists() and (changed or missing):
        raise ValueError("App has a process receipt. Run sdk.ps1 shutdown before assembling.")
    for name in dict.fromkeys(changed + missing):
        path = inside(app, name)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(entries[name])
    previous_path.write_text(json.dumps({"schema": 1, "files": current, "owners": owners}, indent=2), encoding="utf-8")
    return {"ok": True, "file_count": len(entries), "changed": len(set(changed + missing)), "app": str(app)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--export", action="store_true", help="Export editable packages; exclude generated app/ and environment")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if args.export:
        stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y-%m-%d-%H-%M")
        output = root.parent / f"{stamp}-Quest_E6_SDK_Edited.zip"
        release_manifest = root / "RELEASE_FILES.json"
        reviewed = json.loads(release_manifest.read_text(encoding="utf-8"))["files"]
        for name, expected in reviewed.items():
            if digest(inside(root, name).read_bytes()) != expected:
                raise ValueError(f"Changed public source: {name}. Review changes before creating a new public release.")
        with zipfile.ZipFile(output, "x", zipfile.ZIP_DEFLATED) as archive:
            for name in [*reviewed, "RELEASE_FILES.json"]:
                archive.write(inside(root, name), "Quest_E6_SDK/" + name)
        print(json.dumps({"archive": str(output), "hardware_validation": False}, indent=2))
    else:
        print(json.dumps(assemble(root, check=args.check), indent=2))
