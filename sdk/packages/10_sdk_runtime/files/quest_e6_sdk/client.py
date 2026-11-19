"""Small synchronous loopback HTTP client; no direct robot sockets or retries."""

import json
import math
from http.client import HTTPConnection, HTTPException
from urllib.parse import urlencode

PROJECT_ID = "quest3-dual-e6-readonly-20261029"
MAX_RESPONSE_BYTES = 4 * 1024 * 1024


class SDKError(RuntimeError):
    """Preserves backend rejection details and ambiguous transport outcomes."""

    def __init__(self, message, *, status=None, outcome_unknown=False):
        super().__init__(message)
        self.status = status
        self.outcome_unknown = outcome_unknown


class TeleopClient:
    """Use the running PC service without taking ownership of either E6 socket.

    Methods return original backend dictionaries. HTTP 200 is not proof of
    physical success: inspect state, reason, per-arm feedback and stop_evidence.
    An ambiguous POST failure is never retried automatically.
    """

    def __init__(self, *, port=8765, timeout_s=45.0):
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("port must be an integer in 1..65535")
        if type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be finite and positive")
        self.port, self.timeout_s = port, timeout_s

    def _request(self, method, path, *, query=None, body=None):
        if query:
            path += "?" + urlencode(query)
        data = None if body is None else json.dumps(body, allow_nan=False).encode("utf-8")
        connection = HTTPConnection("127.0.0.1", self.port, timeout=self.timeout_s)
        try:
            connection.request(method, path, body=data, headers={
                "Content-Type": "application/json", "Cache-Control": "no-cache",
            })
            response = connection.getresponse()
            raw = response.read(MAX_RESPONSE_BYTES + 1)
            if len(raw) > MAX_RESPONSE_BYTES:
                raise SDKError("Response exceeds 4 MiB", outcome_unknown=method != "GET")
            text = raw.decode("utf-8", "replace")
            if not 200 <= response.status < 300:
                raise SDKError(f"HTTP {response.status}: {text}", status=response.status)
            try:
                result = json.loads(text)
            except ValueError as exc:
                raise SDKError("Backend did not return JSON", outcome_unknown=method != "GET") from exc
            if not isinstance(result, dict):
                raise SDKError("Expected a JSON object", outcome_unknown=method != "GET")
            return result
        except (OSError, HTTPException) as exc:
            raise SDKError(
                f"{method} {path}: {exc}. No retry was made.",
                outcome_unknown=method != "GET",
            ) from exc
        finally:
            connection.close()

    @staticmethod
    def _side(side, *, auto=False):
        allowed = ("left", "right", "both", "auto") if auto else ("left", "right", "both")
        if side not in allowed:
            raise ValueError(f"side must be one of {allowed}")
        return {"side": side}

    def _post(self, path, *, query=None, body=None):
        # Check the service identity, without changing robot state.
        self.status()
        return self._request("POST", path, query=query, body=body)

    def status(self):
        """Read cached application state; does not initiate robot connections."""
        result = self._request("GET", "/api/status")
        if result.get("project_id") != PROJECT_ID:
            raise SDKError("Unexpected service project_id; no control request was sent")
        return result

    def quest(self):
        """Read actual received input, including stale/disconnected flags."""
        return self.status()["quest"]

    def monitor(self):
        """Read cached per-arm measured feedback and requested/control state."""
        return self._request("GET", "/api/monitor/status")

    def headset_status(self):
        return self._request("GET", "/api/headset/status")

    def lan_info(self):
        """Read current URL/pair code. Treat the result as session-local data."""
        return self._request("GET", "/api/quest/lan")

    def enable_lan(self):
        """Start the listener using explicitly prepared runtime TLS files."""
        return self._post("/api/quest/lan/enable")

    def renew_pairing(self):
        return self._post("/api/quest/lan/renew")

    def connect_usb(self):
        return self._post("/api/quest/connect-usb")

    def connect_feedback(self, side="both"):
        """Open/reuse passive 30004 readers; sends no robot motion command."""
        return self._post("/api/monitor/connect", query=self._side(side))

    def disconnect_feedback(self, side="both"):
        return self._post("/api/monitor/disconnect", query=self._side(side))

    def check_robot(self, side="both"):
        """One-shot read-only diagnostics; backend rejects conflicting owners."""
        return self._post("/api/e6/check", query=self._side(side))

    def prepare(self, side="auto"):
        """Identify/query selected arms and prepare models; does not arm motion."""
        return self._post("/api/teleop/prepare", query=self._side(side, auto=True))

    def arm(self, side="auto"):
        """Explicitly enable following permission via existing backend checks.

        Requires prepared participants, valid live AR, calibration and start
        pose. Actual motion can follow subsequent Grip input. Never auto-retry.
        """
        return self._post("/api/teleop/arm", query=self._side(side, auto=True))

    def stop(self):
        """Request group/home Stop; inspect every arm's feedback confirmation."""
        return self._post("/api/teleop/stop")

    def capture_diagnostic(self):
        """Request the existing short diagnostic recording, without arming."""
        return self._post("/api/recording/capture")

    def save_quest_observation(self):
        return self._post("/api/quest/save")

    def capture_start(self, side, *, client_id):
        """Save measured start state; use a stable caller ID for audit context."""
        if side not in ("left", "right"):
            raise ValueError("capture_start requires left or right")
        return self._post("/api/start-positions/action", body={
            "action": "capture", "side": side, "client_id": client_id,
        })

    def return_start(self, side, *, client_id):
        """Request actual return motion; caller MUST maintain home_heartbeat.

        Returns an accepted asynchronous operation, not completed movement.
        Current backend may explicitly recover collision protection and enable
        this arm as part of return. See docs/SDK_API.md before using this method.
        The provided PC/VR UI already implements the owner/heartbeat lifecycle.
        """
        if side not in ("left", "right"):
            raise ValueError("return_start requires left or right")
        return self._post("/api/start-positions/action", body={
            "action": "return", "side": side, "client_id": client_id,
        })

    def home_heartbeat(self, operation_id, *, client_id):
        # The caller owns lifecycle/timing; there is no hidden background loop.
        return self._request("POST", "/api/start-positions/heartbeat", body={
            "operation_id": operation_id, "client_id": client_id,
        })
