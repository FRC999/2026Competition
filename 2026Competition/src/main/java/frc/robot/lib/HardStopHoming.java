package frc.robot.lib;

/** Bounded stall-based homing: timeout is failure, and only continuous fresh evidence qualifies. */
public final class HardStopHoming {
  public enum Result { MOVING, CONFIRMED_STALL, TIMED_OUT }
  private final double minimum, confirm, timeout;
  private double start, evidenceSince = Double.NaN;
  private Result result = Result.MOVING;
  /** Durations in seconds: ignore early contact, then require continuous evidence before the hard cap. */
  public HardStopHoming(double minimum, double confirm, double timeout) {
    this.minimum = minimum; this.confirm = confirm; this.timeout = timeout;
  }
  /** Starts a new attempt using a monotonic clock, normally FPGA seconds. */
  public void start(double now) { start = now; evidenceSince = Double.NaN; result = Result.MOVING; }
  /** Terminal results stay latched until start(); stale/absent evidence resets the confirmation timer. */
  public Result update(double now, boolean freshStallEvidence) {
    if (result != Result.MOVING) return result;
    if (now - start >= timeout) { result = Result.TIMED_OUT; return result; }
    if (freshStallEvidence && now - start >= minimum) {
      if (!Double.isFinite(evidenceSince)) evidenceSince = now;
      if (now - evidenceSince >= confirm) result = Result.CONFIRMED_STALL;
    } else evidenceSince = Double.NaN;
    return result;
  }
  public Result result() { return result; }
}
