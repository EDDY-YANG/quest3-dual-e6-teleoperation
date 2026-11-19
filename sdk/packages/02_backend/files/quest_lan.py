"""Paired Quest HTTPS/WSS; generic robot/ADB HTTP APIs remain loopback-only."""

import ipaddress
import json
import math
import secrets
import ssl
import time
from collections import defaultdict, deque
from html import escape

from aiohttp import web
from web_assets import robot_asset_response
from asset_registry import register_asset_routes

PAIR_CODE_SECONDS = 3600


def pairing_page(message=""):
    return f"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pair Quest input</title><link rel="stylesheet" href="/static/style.css"></head>
<body class="quest-page"><main><h1>Pair Quest 3</h1>
<p>Enter the CURRENT six-digit code provided by the PC service. After entering AR, use the headset panel to prepare, calibrate and explicitly start teleoperation.</p>
<p role="alert">{escape(message)}</p>
<form action="/pair" method="post"><label for="code">Pairing code</label><br>
<input id="code" name="code" type="text" inputmode="numeric" pattern="[0-9]{{6}}"
 minlength="6" maxlength="6" required autocomplete="off"
 style="font-size:32px;width:240px;padding:12px;margin:16px 0"><br>
<button>Connect input</button></form><p><a href="/">Return to pairing page</a></p>
<p>After pairing, choose Start AR / Passthrough.</p></main></body></html>"""


class QuestLan:
    def __init__(self, root, session_handler, websocket_handler):
        self.root = root
        self.session_handler = session_handler
        self.websocket_handler = websocket_handler
        self.runner = None
        self.info = {"enabled": False}
        self.code = None
        self.cookie = None
        self.expires = 0
        self.attempts = defaultdict(deque)

    def snapshot(self):
        remaining = max(0, math.ceil(self.expires - time.monotonic()))
        return {**self.info, "code_remaining_seconds": remaining, "code_expired": remaining == 0}

    def code_error(self, submitted, now=None):
        now = time.monotonic() if now is None else now
        if now >= self.expires:
            return "Code expired. Generate a new code on the PC, then enter it here."
        code = str(submitted).strip()
        if not code.isascii() or not secrets.compare_digest(code, self.code):
            return "Code does not match. Enter the current six-digit code shown on the PC."
        return None

    async def status_handler(self, request):
        return web.json_response(self.snapshot())

    async def renew(self):
        if not self.runner:
            config = json.loads((self.root / "runtime/lan-config.json").read_text())
            await self.start(config)
        else:
            self.code = f"{secrets.randbelow(1000000):06d}"
            self.cookie = secrets.token_urlsafe(32)
            self.expires = time.monotonic() + PAIR_CODE_SECONDS
            self.attempts.clear()
            self.info.update(
                pair_code=self.code, paired_peer=None, code_valid_seconds=PAIR_CODE_SECONDS
            )
        return web.json_response(self.snapshot())

    async def enable_handler(self, request):
        if self.runner:
            return web.json_response(self.snapshot())
        config = json.loads((self.root / "runtime/lan-config.json").read_text())
        await self.start(config)
        return web.json_response(self.snapshot())

    async def start(self, config):
        address = ipaddress.IPv4Address(config["bind_ip"])
        network = ipaddress.IPv4Network(config["subnet"], strict=False)
        if not address.is_private or address.is_loopback or address not in network:
            raise ValueError("LAN listener requires a private interface address in its subnet")
        authority = f"{address}:8443"
        origin = f"https://{authority}"
        self.code = f"{secrets.randbelow(1000000):06d}"
        self.cookie = secrets.token_urlsafe(32)
        self.expires = time.monotonic() + PAIR_CODE_SECONDS
        self.attempts.clear()
        self.info = {
            "enabled": False,
            "url": origin,
            "pair_code": self.code,
            "code_valid_seconds": PAIR_CODE_SECONDS,
            "last_request_at": None,
            "paired_peer": None,
            "message": "Paired Quest input and explicit trial requests; generic robot/ADB APIs remain loopback-only.",
        }

        @web.middleware
        async def guard(request, handler):
            try:
                peer = ipaddress.ip_address(request.remote)
            except ValueError:
                raise web.HTTPForbidden()
            if peer not in network or request.host != authority:
                raise web.HTTPForbidden()
            if request.headers.get("Origin") not in (None, origin):
                raise web.HTTPForbidden(text="Unexpected origin")
            self.info["last_request_at"] = time.time()
            authenticated = secrets.compare_digest(
                request.cookies.get("quest_input", ""), self.cookie
            )
            if request.path not in ("/", "/pair", "/static/style.css") and not authenticated:
                if request.path == "/quest":
                    raise web.HTTPFound("/")
                raise web.HTTPUnauthorized(text="Pair this headset first at /")
            response = await handler(request)
            response.headers["Cache-Control"] = "no-store"
            response.headers["X-Content-Type-Options"] = "nosniff"
            return response

        async def landing(request):
            if secrets.compare_digest(request.cookies.get("quest_input", ""), self.cookie):
                raise web.HTTPFound("/quest")
            return web.Response(text=pairing_page(), content_type="text/html")

        async def pair(request):
            now = time.monotonic()
            attempts = self.attempts[request.remote]
            while attempts and attempts[0] < now - 60:
                attempts.popleft()
            if len(attempts) >= 5:
                return web.Response(
                    text=pairing_page("Too many attempts. Wait one minute before trying again."),
                    status=429,
                    content_type="text/html",
                )
            attempts.append(now)
            data = await request.post()
            error = self.code_error(data.get("code", ""), now)
            if error:
                return web.Response(text=pairing_page(error), status=403, content_type="text/html")
            if self.info["paired_peer"] not in (None, request.remote):
                raise web.HTTPConflict(
                    text="Another device is paired; restart the LAN listener to replace it"
                )
            self.info["paired_peer"] = request.remote
            response = web.HTTPFound("/quest")
            response.set_cookie(
                "quest_input",
                self.cookie,
                secure=True,
                httponly=True,
                samesite="Strict",
                max_age=7200,
            )
            raise response

        async def static(request):
            name = request.match_info["name"]
            if name not in ("quest.js", "grip_gate.mjs", "quest_hud.mjs", "style.css", "robot_view.mjs", "direction_calibration.mjs", "tracking_warning.mjs", "e6_urdf_view.mjs", "start_positions.mjs", "asset_store.mjs", "gripper_model.mjs", "gripper_ui_input.mjs"):
                raise web.HTTPNotFound()
            return web.FileResponse(self.root / "web" / name)

        async def quest_page(request):
            return web.FileResponse(self.root / "web/quest.html")

        async def robot_asset(request):
            return robot_asset_response(self.root, request)

        lan_app = web.Application(middlewares=[guard], client_max_size=16384)
        register_asset_routes(lan_app, self.root)
        lan_app.add_routes(
            [
                web.get("/", landing),
                web.get("/static/robot-assets/{package}/{asset}", robot_asset),
                web.get("/pair", landing),
                web.post("/pair", pair),
                web.get("/quest", quest_page),
                web.get("/static/{name}", static),
                web.get("/api/session", self.session_handler),
                web.get("/ws/quest", self.websocket_handler),
            ]
        )
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.load_cert_chain(
            self.root / "runtime/lan-cert.pem", self.root / "runtime/lan-key.pem"
        )
        runner = web.AppRunner(lan_app, access_log=None)
        await runner.setup()
        try:
            await web.TCPSite(runner, str(address), 8443, ssl_context=context).start()
        except Exception:
            await runner.cleanup()
            raise
        self.runner = runner
        self.info["enabled"] = True

    async def cleanup(self, app):
        if self.runner:
            await self.runner.cleanup()
            self.runner = None
