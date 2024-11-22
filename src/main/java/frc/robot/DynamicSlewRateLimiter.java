package frc.robot;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class DynamicSlewRateLimiter {
  private final double increasingRateLimit;
  private final double decreasingRateLimit;
  private double prevVal;
  private double prevTime;

  /**
   * Creates a new DynamicSlewRateLimiter with the given increasing and decreasing rate limits.
   *
   * The rate limits are only magnitudes.
   * @param increasingRateLimit The rate-of-change limit when the input is increasing, in units per
   *     second. This is expected to be positive. How quickly the input can accelerate.
   * @param decreasingRateLimit The rate-of-change limit when the input is decreasing, in units per
   *     second. This is expected to be positive. How quickly the input can decelerate.
   */
  public DynamicSlewRateLimiter(double increasingRateLimit, double decreasingRateLimit) {
    this.increasingRateLimit = increasingRateLimit;
    this.decreasingRateLimit = decreasingRateLimit;
    prevVal = 0;
    prevTime = MathSharedStore.getTimestamp();
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - prevTime;
    double sign = Math.signum(prevVal);
    double positiveRateLimit = increasingRateLimit;
    double negativeRateLimit = decreasingRateLimit;
    // Flip the positive and negative limits so that decreasing still means towards zero and increasing still means away.
    if (sign < 0) { 
        positiveRateLimit = decreasingRateLimit;
        negativeRateLimit = increasingRateLimit;
    } 
    prevVal +=
        MathUtil.clamp(
            input - prevVal,
            -negativeRateLimit * elapsedTime,
            positiveRateLimit * elapsedTime);
    prevTime = currentTime;
    SmartDashboard.putNumber("positiveRateLimit", positiveRateLimit);
    SmartDashboard.putNumber("negativeRateLimit", negativeRateLimit);
    SmartDashboard.putNumber("sign", sign);
    return prevVal;
  }

  /**
   * Returns the value last calculated by the DynamicSlewRateLimiter.
   *
   * @return The last value.
   */
  public double lastValue() {
    return prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    prevVal = value;
    prevTime = MathSharedStore.getTimestamp();
  }

}
