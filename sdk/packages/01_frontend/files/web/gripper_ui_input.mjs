// A menu click starts on a button. A configured independent jaw also receives
// the analog trigger throughout that gesture, regardless of where the ray points.
export class TriggerOwnership {
  constructor() {
    this.hands = {};
    this.captured = {};
    this.dedicated = {left:false, right:false};
    this.reset();
  }
  reset(side) {
    for (const hand of side ? [side] : ["left", "right"]) {
      this.hands[hand] = {needRelease:true, owner:null, button:null, clicked:false, releasingJaw:false, value:null, faceDown:null};
      this.captured[hand] = true;
    }
  }
  setDedicated(side, enabled) {
    if (this.dedicated[side] === enabled) return;
    this.dedicated[side] = enabled;
    this.reset(side);
  }
  sample(side, value, ray, pressed=false, facePressed=null) {
    const wasBlocked = this.captured[side];
    if (!Number.isFinite(value) || value < 0 || value > 1) {
      this.reset(side);
      return {blocked:true, click:null, sendNow:!wasBlocked};
    }
    const hand = this.hands[side];
    const independent = this.dedicated[side];
    let blocked = true, click = null, sendNow = false;
    if (value <= .05) {
      if (ray || independent) hand.needRelease = false;
      if (hand.owner === "jaw") {
        hand.releasingJaw = true;
        sendNow = true;
      }
      // Forward the last few analog samples too, e.g. .025 -> 0 over a button.
      if (hand.releasingJaw && value === 0 && hand.value > 0) sendNow = true;
      hand.owner = null;
      blocked = hand.needRelease || (!hand.releasingJaw && (!ray || ray.hit));
    } else if (!hand.needRelease) {
      if (hand.owner === null) {
        if (ray || independent) {
          hand.owner = ray?.button || (!independent && ray?.hit) ? "hud" : "jaw";
          hand.button = ray?.button ?? null;
          hand.clicked = false;
          hand.releasingJaw = false;
        } else {
          hand.needRelease = true;
          hand.releasingJaw = false;
        }
      }
      blocked = hand.owner !== "jaw";
      if (hand.owner === "hud" && !hand.clicked && (pressed || value >= .7)) {
        hand.clicked = true;
        if (ray?.button === hand.button) click = hand.button;
      }
    }
    if (independent) {
      blocked = hand.needRelease;
      if (value === 0 && hand.value > 0) sendNow = true;
    }
    if (facePressed === true && hand.faceDown === false && ray?.button) click = ray.button;
    hand.faceDown = facePressed;
    hand.value = value;
    this.captured[side] = blocked;
    return {blocked, click, sendNow:sendNow || blocked !== wasBlocked};
  }
  update(side, value, ray) {
    return this.sample(side, value, ray).blocked;
  }
}
