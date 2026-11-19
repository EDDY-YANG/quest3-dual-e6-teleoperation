// A trigger aimed at the central HUD belongs to UI until the analog value is released.
export class TriggerOwnership {
  constructor() { this.reset(); }
  reset() { this.captured={left:true,right:true}; }
  update(side,value,ray) {
    if (!ray || !Number.isFinite(value) || value < 0 || value > 1) return this.captured[side]=true;
    if (ray.hit) return this.captured[side]=true;
    if (value <= .05) this.captured[side]=false;
    return this.captured[side];
  }
}
