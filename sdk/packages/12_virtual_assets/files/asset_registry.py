"""Single on-disk catalog for the PC, Quest LAN, renderer and dataset metadata."""
import json
from functools import lru_cache
from pathlib import Path
from aiohttp import web

ROOT = Path(__file__).resolve().parent
@lru_cache(maxsize=4)
def load_catalog(root=ROOT):
    data=json.loads((Path(root)/"web/asset_catalog.json").read_text(encoding="utf-8"))
    if data["schema"] != 1:
        raise ValueError("Unknown virtual asset catalog schema")
    for key, desc in data["assets"].items():
        if not key or Path(key).name != key or key in (".",".."):
            raise ValueError("Invalid asset id")
        for name in desc["files"]:
            if not name or Path(name).name != name or name in (".","..") or ":" in name:
                raise ValueError("Invalid asset filename")
    for side in ("left","right"):
        m=data["instances"][side]
        for k in ("arm","controller","tool"):
            if m[k] not in data["assets"]:raise ValueError("Unknown asset in instance")
    return data

def asset_path(root,key,name):
    desc=load_catalog(root)["assets"].get(key)
    if not desc or name not in desc["files"]:
        raise web.HTTPNotFound()
    return Path(root)/"web/robot-assets"/key/name

def register_asset_routes(app,root):
    async def catalog(request):
        return web.json_response(load_catalog(root))
    async def resource(request):
        return web.FileResponse(asset_path(root,request.match_info["asset_id"],request.match_info["name"]))
    app.router.add_get("/api/assets", catalog)
    app.router.add_get("/api/assets/{asset_id}/{name}", resource)
