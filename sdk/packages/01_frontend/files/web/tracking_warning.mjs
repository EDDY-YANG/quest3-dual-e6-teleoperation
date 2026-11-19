// Nonblocking display and optional, throttled browser speech. No control actions.
export function trackingWarningView(view) {
  const w = view?.control?.tracking_warning;
  if (!view?.fresh || w?.active !== true || w.code !== "TRACKING_LAG_OVER_THRESHOLD" ||
      !Number.isFinite(w.threshold_deg) || !Number.isFinite(w.max_error_deg)) return {active: false};
  const errors = Array.isArray(w.joint_errors_deg) ? w.joint_errors_deg : [];
  const joints = Array.isArray(w.joints_exceeding) ? w.joints_exceeding.filter(j =>
    Number.isInteger(j) && j >= 1 && j <= 6 && Number.isFinite(errors[j - 1])) : [];
  const details = joints.map(j => `J${j} ${Math.abs(errors[j - 1]).toFixed(1)}°`).join(" / ");
  const worst = Number.isInteger(w.worst_joint) && w.worst_joint >= 1 && w.worst_joint <= 6 ?
    w.worst_joint : null;
  return {active: true, nonblocking: w.blocking === false, threshold: w.threshold_deg,
    maxError: w.max_error_deg, worstJoint: worst, joints,
    text: `跟踪偏差提示 · ${details || `最大 ${w.max_error_deg.toFixed(1)}°`} · ${
      w.blocking === false ? "不阻断跟随" : "以后台状态为准"}`,
    speech: `跟踪偏差提示。${worst ? `第${worst}关节` : "最大偏差"}${Math.round(w.max_error_deg)}度。`};
}

export class TrackingAnnouncer {
  constructor({engine = globalThis.speechSynthesis, Utterance = globalThis.SpeechSynthesisUtterance} = {}) {
    this.engine = engine; this.Utterance = Utterance;
    this.supported = Boolean(engine?.speak && Utterance);
    this.enabled = false; this.status = this.supported ? "OFF" : "UNSUPPORTED";
    this.lastSpokenAt = -Infinity; this.lastSpoken = null; this.wasActive = false; this.lastPollAt = -Infinity;
  }
  toggle() {
    if (!this.supported || this.status === "UNAVAILABLE") return false;
    this.enabled = !this.enabled; this.status = this.enabled ? "ON" : "OFF";
    return this.enabled;
  }
  snapshot() { return {supported: this.supported, enabled: this.enabled, status: this.status}; }
  update(warning, now) {
    if (!Number.isFinite(now) || now < this.lastPollAt) return false;
    this.lastPollAt = now;
    const entering = warning.active && !this.wasActive;
    this.wasActive = Boolean(warning.active);
    if (!this.enabled || !warning.active || warning.nonblocking !== true) return false;
    const changed = !this.lastSpoken || this.lastSpoken.worstJoint !== warning.worstJoint ||
      Math.abs(this.lastSpoken.maxError - warning.maxError) >= 10;
    if (!(entering || changed) || now - this.lastSpokenAt < 10000 ||
        this.engine.speaking || this.engine.pending) return false;
    try {
      const utterance = new this.Utterance(warning.speech);
      utterance.lang = "zh-CN"; utterance.rate = 1.0;
      utterance.onerror = () => { this.enabled = false; this.status = "UNAVAILABLE"; };
      this.engine.speak(utterance);
      this.lastSpokenAt = now;
      this.lastSpoken = {worstJoint: warning.worstJoint, maxError: warning.maxError};
      return true;
    } catch {
      this.enabled = false; this.status = "UNAVAILABLE"; return false;
    }
  }
}
