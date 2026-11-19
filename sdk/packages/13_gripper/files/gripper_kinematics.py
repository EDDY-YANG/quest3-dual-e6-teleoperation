"""Calibrated-reference math for the symmetric crank/rod/slide gripper. No I/O."""
import math

def finite_unit(value):
    return type(value) in (int, float) and math.isfinite(value) and 0 <= value <= 1

class ParallelJaw:
    def __init__(self, spec):
        self.cx, self.cy = spec["crank_pin_rest_mm"]
        self.sx, self.sy = spec["slide_pin_rest_mm"]
        self.rod = spec["rod_length_mm"]
        self.closed_gap = spec["closed_tip_clearance_mm"]
        self.swing = math.radians(spec["open_crank_delta_deg"])
        self.rod_rest = math.atan2(self.sy-self.cy, self.sx-self.cx)
        samples=[self._slide(self.swing*i/64)[2] for i in range(65)]
        if abs(samples[0]) > 1e-4 or any(b<=a for a,b in zip(samples,samples[1:])):
            raise ValueError("Invalid or non-monotonic gripper working arc")
        self.travel=samples[-1]

    def _slide(self, delta):
        c,s=math.cos(delta),math.sin(delta)
        x,y=self.cx*c-self.cy*s,self.cx*s+self.cy*c
        span=self.rod**2-(self.sy-y)**2
        if span <= 0: raise ValueError("Gripper linkage outside assembly branch")
        slider_x=x-math.sqrt(span)
        return x,y,self.sx-slider_x

    def pose(self, opening):
        if not finite_unit(opening):raise ValueError("opening must be a finite fraction in [0,1]")
        lo,hi=0.0,self.swing
        for _ in range(32):
            mid=(lo+hi)/2
            if self._slide(mid)[2] < opening*self.travel:lo=mid
            else:hi=mid
        crank=0.0 if opening==0 else self.swing if opening==1 else (lo+hi)/2
        cx,cy,slide=self._slide(crank)
        angle=math.atan2(self.sy-cy,self.sx-slide-cx)-self.rod_rest
        angle=math.atan2(math.sin(angle),math.cos(angle))
        c,s=math.cos(angle),math.sin(angle)
        return {"crank_rad":crank,"crank_fraction":crank/self.swing,"slide_mm":slide,
                "rod_rad":angle,"rod_offset_mm":[cx-(c*self.cx-s*self.cy),cy-(s*self.cx+c*self.cy)],
                "model_gap_mm":self.closed_gap+2*slide}
