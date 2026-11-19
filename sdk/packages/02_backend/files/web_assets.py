"""Explicit public visualization assets; no arbitrary filesystem serving."""

from pathlib import Path
from aiohttp import web
from asset_registry import asset_path, load_catalog

# Compatibility for older client URLs; this allowlist is derived from the same catalog.
ROBOT_ASSETS = {k:frozenset(d["files"]) for k,d in load_catalog()["assets"].items()}

def robot_asset_response(root, request):
    return web.FileResponse(asset_path(root, request.match_info["package"], request.match_info["asset"]))
