// Real poses are always retained. Grip controls motion permission only.
export class GripGate {
  constructor() { this.reset(); }

  reset() {
    this.arms = Object.fromEntries(["left", "right"].map(side =>
      [side, { held: false, needsRelease: true }]));
  }

  update(hands, visible) {
    const output = { left: null, right: null };
    let changed = false;
    for (const side of ["left", "right"]) {
      const hand = hands[side];
      const state = this.arms[side];
      const previous = state.held;
      const value = hand?.gamepad?.buttons?.[1]?.value;
      const tracked = Boolean(visible && hand?.is_controller && hand.pose &&
        hand.pose.emulated_position === false && hand.gamepad?.connected &&
        hand.gamepad.mapping === "xr-standard" && Number.isFinite(value) &&
        value >= 0 && value <= 1);
      if (!tracked) {
        state.held = false;
        state.needsRelease = true;
      } else if (value <= 0.2) {
        state.held = false;
        state.needsRelease = false;
      } else if (!state.needsRelease && value >= 0.7) {
        state.held = true;
      }
      changed ||= previous !== state.held;
      if (hand) output[side] = {
        ...hand,
        tracking_valid: tracked,
        grip_held: state.held,
        pose: hand.pose,
      };
    }
    return { hands: output, changed,
      active: Object.values(this.arms).some(state => state.held) };
  }
}
