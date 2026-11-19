"""Load the delivered native solver and serve actual assets on an isolated port.

No production port is used and robot connections are forbidden. Checks package
relocation/ABI/HTTP assets, not physical motion or a headset browser session.
"""
import asyncio
import json
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def block_robot_connections(event, args):
    if event in ("socket.connect", "socket.connect_ex", "socket.sendto"):
        address = args[-1]
        if not isinstance(address, tuple) or address[0] not in ("127.0.0.1", "::1"):
            raise RuntimeError("Delivery verification forbids non-loopback/robot network I/O")


sys.addaudithook(block_robot_connections)


async def verify_http(temporary):
    from aiohttp import ClientSession
    from aiohttp.test_utils import TestServer
    import server
    from web_assets import ROBOT_ASSETS

    with patch("dobot_safe_sdk.BoundDobotDashboard", side_effect=AssertionError("No robot driver allowed")) as driver:
        app = server.make_app(recording_enabled=False,
                              ranged_profile_path=ROOT / "profiles/rangedik_world_anchor.json",
                              start_positions_path=temporary / "starts.json",
                              calibration_log_dir=temporary / "calibration",
                              trial_runtime_dir=temporary / "native")
        http = TestServer(app, host="127.0.0.1")
        await http.start_server()
        try:
            routes = ["/", "/quest", "/monitor", "/hud-preview", "/api/status"]
            routes += ["/static/" + p.name for p in (ROOT / "web").iterdir()
                       if p.suffix in (".js", ".mjs", ".css")]
            routes += [f"/static/robot-assets/{package}/{asset}"
                       for package, assets in ROBOT_ASSETS.items() for asset in sorted(assets)]
            with patch.object(server, "PORT", http.port):
                async with ClientSession() as session:
                    for route in routes:
                        async with session.get(http.make_url(route)) as response:
                            data = await response.read()
                            if response.status != 200 or not data:
                                raise RuntimeError(f"Asset/HTTP failed: {route} {response.status}")
                            if route == "/api/status":
                                state = json.loads(data)
                                if state["teleop"]["armed"] or state["teleop"]["participants"]:
                                    raise RuntimeError("Unexpected motion participation on isolated startup")
            driver.assert_not_called()
            return {"routes_checked": len(routes), "robot_driver_constructed": False,
                    "configured_profile": state["configured_ranged_ik"]["profile_id"]}
        finally:
            await http.close()


def verify_native(temporary):
    from nominal_model import from_urdf
    from kinematics_validation import pose_error
    from ranged_ik_adapter import RangedIKAdapter
    from ranged_ik_profile import RangedIKProfile

    model = from_urdf(ROOT / "web/robot-assets/e6/me6_robot.urdf")
    profile = RangedIKProfile.load(ROOT / "profiles/rangedik_world_anchor.json")
    q = [13., -24., 31., 9., -18., 27.]  # Explicit model probe, never a measured physical state.
    adapter = RangedIKAdapter(model, profile, q, temporary / "native_probe")
    try:
        goal = model.fk(q)
        error = pose_error(goal, adapter.native_fk(q))
        if max(error) > .05:
            raise RuntimeError("Native FK and model disagree")
        goal[0] += 2.0
        proposal = adapter.propose(goal, q)
        if proposal["status"] not in ("WITHIN_RANGE", "APPROACHING"):
            raise RuntimeError("Native solver did not produce useful progress")
        dls = model.solve(goal, q)
        if not dls["success"]:
            raise RuntimeError("Delivered traditional DLS did not solve its nominal pose")
        return {"model_source": "PUBLIC_DOBOT_URDF_NOMINAL_FLANGE", "native_fk_error_mm_deg": error, "native_status": proposal["status"],
                "traditional_dls_success": dls["success"], "committed_as_robot_ack": False}
    finally:
        adapter.close()


def main():
    with tempfile.TemporaryDirectory(prefix="quest_sdk_delivery_") as temporary:
        directory = Path(temporary)
        result = {"scope": "ISOLATED_HTTP_AND_REAL_NATIVE_OFFLINE_ONLY", "app_root": str(ROOT),
                  "native": verify_native(directory), "http": asyncio.run(verify_http(directory)),
                  "hardware_acceptance": False}
        print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
