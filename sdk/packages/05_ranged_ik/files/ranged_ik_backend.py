"""Typed C ABI for the pinned, patched official RangedIK/PANOC library.

The DLL contains real upstream RangedIK objective/PANOC code. There is no
fallback numerical solver and no robot socket in this module.
"""

import ctypes
import hashlib
import json
import math
import os
import threading
from pathlib import Path

from ranged_ik_profile import vector


class RangedIKBackendError(RuntimeError):
    pass


class NativeRangedIK:
    upstream_commit = "1c48d2ae408b4e024ee037641aac1e728267984e"

    def __init__(self, library_path, settings_path, objective_weights, *, orientation_error_metric="ROTATION_VECTOR_COMPONENTS"):
        path = Path(library_path).resolve()
        if not path.is_file():
            raise RangedIKBackendError(
                f"RANGEDIK_LIBRARY_MISSING: {path}; run the native build helper"
            )
        self.library_sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        manifest_path = path.with_name("build-manifest.json")
        if not manifest_path.is_file():
            raise RangedIKBackendError(
                "RANGEDIK_BUILD_MANIFEST_MISSING: rebuild with build_native.ps1"
            )
        manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
        if (
            manifest.get("dll_sha256") != self.library_sha256
            or manifest.get("source", {}).get("commit") != self.upstream_commit
        ):
            raise RangedIKBackendError("RANGEDIK_BUILD_MANIFEST_MISMATCH")
        self._lock = threading.RLock()
        self._handle = None
        self._dll_directory = os.add_dll_directory(str(path.parent)) if os.name == "nt" else None
        self.lib = ctypes.CDLL(str(path))
        ptr = ctypes.c_void_p
        array = ctypes.POINTER(ctypes.c_double)
        self.lib.rik_new.argtypes = [ctypes.c_char_p]
        self.lib.rik_new.restype = ptr
        self.lib.rik_last_error.argtypes = []
        self.lib.rik_last_error.restype = ctypes.c_char_p
        self.lib.rik_free.argtypes = [ptr]
        self.lib.rik_free.restype = None
        for name, args in {
            "rik_reset": [ptr, array],
            "rik_configure": [ptr, array],
            "rik_solve_position": [ptr] + [array] * 7,
            "rik_fk": [ptr, array, array],
            "rik_stats": [ptr, array],
            "rik_commit": [ptr, array],
        }.items():
            fn = getattr(self.lib, name)
            fn.argtypes = args
            fn.restype = ctypes.c_int32
        batch_exports = {
            "rik_begin_preview_batch": [ptr],
            "rik_get_preview_id": [ptr, ctypes.POINTER(ctypes.c_uint64)],
            "rik_select_preview": [ptr, ctypes.c_uint64],
            "rik_set_solve_budget_ms": [ptr, ctypes.c_double],
        }
        self.has_preview_selection = (
            "preview_batch_selection_and_solver_time_budget" in manifest.get("source",{}).get("patches",[])
            and all(hasattr(self.lib,name) for name in batch_exports))
        if self.has_preview_selection:
            for name,args in batch_exports.items():
                fn=getattr(self.lib,name)
                fn.argtypes=args
                fn.restype=ctypes.c_int32
        self.last_preview_id = None
        self._handle = self.lib.rik_new(str(Path(settings_path).resolve()).encode("utf-8"))
        if not self._handle:
            message = self._error()
            self.close()
            raise RangedIKBackendError(message)
        try:
            if orientation_error_metric not in ("ROTATION_VECTOR_COMPONENTS", "GEODESIC_ANGLE"):
                raise RangedIKBackendError("Invalid orientation error metric")
            metric_fn = getattr(self.lib, "rik_set_orientation_error_metric", None)
            if orientation_error_metric == "GEODESIC_ANGLE" and (
                metric_fn is None or "geodesic_orientation_range"
                not in manifest.get("source", {}).get("patches", [])
            ):
                raise RangedIKBackendError("RANGEDIK_GEODESIC_ABI_MISSING: rebuild native backend")
            center_weight = objective_weights.get("orientation_center", 0.0)
            if type(center_weight) not in (int, float) or not math.isfinite(center_weight) or center_weight < 0:
                raise RangedIKBackendError("Invalid orientation center weight")
            center_fn = getattr(self.lib, "rik_set_orientation_center_weight", None)
            if center_weight > 0 and (
                center_fn is None or "optional_normalized_preferred_orientation_center_bias"
                not in manifest.get("source", {}).get("patches", [])
            ):
                raise RangedIKBackendError("RANGEDIK_ORIENTATION_CENTER_ABI_MISSING: rebuild native backend")
            weights = [*objective_weights["position"], *objective_weights["rotation"]]
            weights += [
                objective_weights[k]
                for k in ("joint_limits", "velocity", "acceleration", "jerk", "manipulability")
            ]
            self._check(self.lib.rik_configure(self._handle, self._array(weights, 11)))
            if metric_fn is not None:
                metric_fn.argtypes = [ptr, ctypes.c_int32]
                metric_fn.restype = ctypes.c_int32
                self._check(metric_fn(self._handle, int(orientation_error_metric == "GEODESIC_ANGLE")))
            self.orientation_error_metric = orientation_error_metric
            if center_fn is not None:
                center_fn.argtypes = [ptr, ctypes.c_double]
                center_fn.restype = ctypes.c_int32
                self._check(center_fn(self._handle, center_weight))
            self.orientation_center_weight = center_weight
        except Exception:
            self.close()
            raise

    @staticmethod
    def _array(values, count):
        return (ctypes.c_double * count)(*vector(values, count, "native argument"))

    def _error(self):
        raw = self.lib.rik_last_error()
        return "RANGEDIK_NATIVE_ERROR: " + (
            raw.decode("utf-8", "replace") if raw else "unspecified"
        )

    def _check(self, code):
        if code != 0:
            raise RangedIKBackendError(self._error())

    def _alive(self):
        if not self._handle:
            raise RangedIKBackendError("RANGEDIK_HANDLE_CLOSED")

    def reset(self, q_deg):
        with self._lock:
            self._alive()
            self._check(
                self.lib.rik_reset(
                    self._handle,
                    self._array([math.radians(x) for x in vector(q_deg, 6, "joints")], 6),
                )
            )

    def begin_preview_batch(self):
        with self._lock:
            self._alive()
            if not self.has_preview_selection:
                raise RangedIKBackendError("RANGEDIK_MULTI_START_ABI_MISSING")
            self._check(self.lib.rik_begin_preview_batch(self._handle))
            self.last_preview_id = None

    def select_preview(self, preview_id):
        with self._lock:
            self._alive()
            if not self.has_preview_selection or type(preview_id) is not int or not 0 < preview_id < 2**64:
                raise RangedIKBackendError("Invalid preview selection")
            self._check(self.lib.rik_select_preview(self._handle, preview_id))
            self.last_preview_id = preview_id
            self._read_stats()

    def _read_stats(self):
        stats = (ctypes.c_double * 5)()
        self._check(self.lib.rik_stats(self._handle, stats))
        self.last_stats = {
            "numerically_converged": bool(stats[0]),
            "iterations": int(stats[1]),
            "native_solve_ms": stats[2],
            "fixed_point_residual": stats[3] if math.isfinite(stats[3]) else None,
            "cost": stats[4] if math.isfinite(stats[4]) else None,
        }

    def propose(self, position_mm, quaternion_xyzw, tolerances_mm_deg, seed_deg, limits_deg, *, time_budget_ms=None):
        with self._lock:
            self._alive()
            budget=80.0 if time_budget_ms is None else time_budget_ms
            if type(budget) not in (int,float) or not math.isfinite(budget) or not 0 < budget <= 80:
                raise RangedIKBackendError("Invalid solve time budget")
            if self.has_preview_selection:
                self._check(self.lib.rik_set_solve_budget_ms(self._handle,budget))
            elif time_budget_ms is not None:
                raise RangedIKBackendError("RANGEDIK_MULTI_START_ABI_MISSING")
            tolerance = vector(tolerances_mm_deg, 6, "tolerances")
            output = (ctypes.c_double * 6)()
            args = [
                self._array([v / 1000 for v in vector(position_mm, 3, "position")], 3),
                self._array(quaternion_xyzw, 4),
                self._array(
                    [v / 1000 for v in tolerance[:3]] + [math.radians(v) for v in tolerance[3:]], 6
                ),
                self._array([math.radians(v) for v in vector(seed_deg, 6, "seed")], 6),
                self._array([math.radians(x[0]) for x in limits_deg], 6),
                self._array([math.radians(x[1]) for x in limits_deg], 6),
            ]
            self._check(self.lib.rik_solve_position(self._handle, *args, output))
            if self.has_preview_selection:
                preview_id=ctypes.c_uint64()
                self._check(self.lib.rik_get_preview_id(self._handle,ctypes.byref(preview_id)))
                self.last_preview_id=preview_id.value
            self._read_stats()
            return list(vector([math.degrees(v) for v in output], 6, "native solution"))

    def fk(self, q_deg):
        with self._lock:
            self._alive()
            output = (ctypes.c_double * 7)()
            q = self._array([math.radians(x) for x in vector(q_deg, 6, "joints")], 6)
            self._check(self.lib.rik_fk(self._handle, q, output))
            values = vector(list(output), 7, "native FK")
            return [v * 1000 for v in values[:3]], list(values[3:])

    def commit(self, q_deg):
        with self._lock:
            self._alive()
            q = self._array([math.radians(x) for x in vector(q_deg, 6, "joints")], 6)
            self._check(self.lib.rik_commit(self._handle, q))

    def close(self):
        with self._lock:
            if self._handle:
                self.lib.rik_free(self._handle)
                self._handle = None
            if self._dll_directory:
                self._dll_directory.close()
                self._dll_directory = None

    def __del__(self):
        if getattr(self, "_handle", None):
            self.close()
