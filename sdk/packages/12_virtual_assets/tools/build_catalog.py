"""Generate the one runtime catalog from reviewed resource names and package manifests."""
from pathlib import Path
import hashlib
import json

package = Path(__file__).resolve().parents[1]
web = package / "files/web"
source = json.loads((package / "catalog.sources.json").read_text(encoding="utf8"))
for asset_id, descriptor in source["assets"].items():
    if Path(asset_id).name != asset_id or asset_id in (".", ".."):
        raise ValueError("Invalid asset ID")
    folder = web / "robot-assets" / asset_id
    names = list(descriptor.pop("include", []))
    manifest = descriptor.pop("resource_manifest", None)
    if manifest:
        entries = json.loads((folder / manifest).read_text(encoding="utf8"))["files"]
        names.extend([manifest, *(e["file"] for e in entries)])
    if len(names) != len(set(names)) or any(Path(n).name != n or n in ("", ".", "..") for n in names):
        raise ValueError("Invalid resource list for " + asset_id)
    descriptor["files"] = {
        name: {"sha256": hashlib.sha256((folder / name).read_bytes()).hexdigest(),
               "bytes": (folder / name).stat().st_size} for name in sorted(names)
    }
source["revision"] = hashlib.sha256(json.dumps(source, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
(web / "asset_catalog.json").write_text(json.dumps(source, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
