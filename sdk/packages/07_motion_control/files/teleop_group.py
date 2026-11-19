"""Coordinate independent arm controllers; no robot sockets or target generation."""
import asyncio
import time
from collections import deque
from contextlib import suppress


class TeleopGroup:
    def __init__(self, event=None):
        self.controllers = {}
        self.participants = ()
        self.state, self.reason = "NOT_PREPARED", None
        self.stop_evidence = None
        self.event = event or (lambda *args: None)
        self.events = deque(maxlen=24)
        self._gate = False
        self._epoch = 0
        self._failure = None
        self._watch_task = self._stop_task = None

    def _event(self, kind, **data):
        record = {"kind": kind, "at": time.monotonic(), **data}
        self.events.append(record)
        self.event("teleop_group", record)

    def _problem(self):
        for side in self.participants:
            trial = self.controllers.get(side)
            if trial is None or not trial.armed or trial.state == "FAULT":
                return {"side": side, "state": getattr(trial, "state", "MISSING"),
                        "reason": getattr(trial, "reason", "CONTROLLER_MISSING")}
        return None

    def motion_allowed(self, side):
        if not self._gate or self.state != "ARMED" or side not in self.participants:
            return False
        problem = self._problem()
        if problem is not None:
            self._failure = self._failure or problem
            return False
        return True

    def require_stopped(self):
        if self._gate or self.state in ("ARMING", "STOPPING") or any(
            t.armed or t.state in ("ARMING", "STOPPING") for t in self.controllers.values()
        ):
            raise ValueError("Stop and confirm all active arms before preparing")
        if self.state == "FAULT" and not (self.stop_evidence or {}).get("feedback_confirmed"):
            raise ValueError("Group Stop has not been confirmed on every arm")
        if any(t.state == "FAULT" and not (t.stop_evidence or {}).get("feedback_confirmed")
               for t in self.controllers.values()):
            raise ValueError("An owned arm still has an unconfirmed Stop")

    def set_prepared(self, sides):
        self.require_stopped()
        sides = tuple(sides)
        if not sides or any(s not in ("left", "right") or s not in self.controllers
                            or self.controllers[s].state != "PREPARED" for s in sides):
            raise ValueError("Every participant must pass its own preflight")
        self.participants = sides
        self._failure = self.stop_evidence = None
        self.state, self.reason = "PREPARED", "EXPLICIT_ARM_REQUIRED"
        self._event("PREPARED", participants=list(sides))

    async def start(self):
        if self._watch_task is None or self._watch_task.done():
            self._watch_task = asyncio.create_task(self._watch(), name="teleop-group-watchdog")

    async def arm(self):
        if self.state != "PREPARED" or not self.participants:
            raise ValueError("Prepare all selected arms first")
        self._gate = False
        self._epoch += 1
        epoch = self._epoch
        self._failure = None
        self.state, self.reason = "ARMING", "WAITING_FOR_ALL_ARMS"
        results = await asyncio.gather(
            *(self.controllers[s].arm() for s in self.participants), return_exceptions=True)
        errors = {s: str(result) for s, result in zip(self.participants, results) if isinstance(result, BaseException)}
        if epoch == self._epoch and not errors:
            for side in self.participants:
                try:
                    self.controllers[side].assert_group_ready()
                except Exception as exc:
                    errors[side] = str(exc)
        if epoch != self._epoch or errors or self._problem():
            if epoch == self._epoch:
                await self.stop("GROUP_ARM_FAILED", details=errors, explicit=False)
            raise ValueError("GROUP_ARM_FAILED: " + (str(errors) if errors else "start cancelled"))
        self.state, self.reason = "ARMED", "INDEPENDENT_GRIP_FOR_EACH_ARM"
        self._gate = True
        self._event("ARMED", participants=list(self.participants))
        await self.start()

    async def stop(self, reason="OPERATOR_DISARMED", details=None, *, explicit=True):
        self._gate = False
        self._epoch += 1
        if self._stop_task is not None and not self._stop_task.done():
            return await self._stop_task
        self._stop_task = asyncio.create_task(self._stop_all(reason, details, explicit), name="teleop-group-stop")
        return await self._stop_task

    async def _stop_all(self, reason, details, explicit):
        self.state, self.reason = "STOPPING", reason
        sides = tuple(self.controllers)
        # Start all Stop operations before waiting for any one transport.
        results = await asyncio.gather(
            *((self.controllers[s].disarm() if explicit else self.controllers[s].fault_stop(reason))
              for s in sides), return_exceptions=True)
        arms = {}
        for side, result in zip(sides, results):
            if isinstance(result, BaseException):
                arms[side] = {"feedback_confirmed": False, "error": str(result)}
            else:
                arms[side] = dict(self.controllers[side].snapshot().get("stop_evidence") or {})
        confirmed = all(a.get("feedback_confirmed") is True for a in arms.values())
        self.stop_evidence = {"feedback_confirmed": confirmed, "arms": arms,
                              "trigger": self._failure, "details": details}
        self.state = "DISARMED" if confirmed else "FAULT"
        self._event("STOP_RESULT", reason=reason, evidence=self.stop_evidence)
        return self.snapshot()

    async def _watch(self):
        while True:
            if self._gate:
                problem = self._failure or self._problem()
                if problem is not None:
                    self._failure = problem
                    await self.stop("PARTICIPANT_FAULT_OR_DISARMED", details=problem, explicit=False)
            await asyncio.sleep(.02)

    def snapshot(self):
        return {"state": self.state, "reason": self.reason, "armed": self._gate,
                "participants": list(self.participants), "mode": "dual" if len(self.participants) == 2 else "single",
                "stop_evidence": self.stop_evidence, "failure": self._failure,
                "events": list(self.events), "start_barrier": "ALL_ARMS_READY_BEFORE_ANY_FOLLOWING"}

    async def close(self):
        if self._gate or any(t.armed for t in self.controllers.values()):
            await self.stop("SERVICE_CLEANUP")
        if self._watch_task:
            self._watch_task.cancel()
            with suppress(asyncio.CancelledError):
                await self._watch_task
