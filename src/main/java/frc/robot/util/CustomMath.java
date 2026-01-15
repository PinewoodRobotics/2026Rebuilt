package frc.robot.util;

/**
 * @note MathFun = Math Functions
 * @apiNote this is the file where all of the math functions go
 */
public class CustomMath {

  /**
   * Wraps an angle to the range [-180, 180] degrees.
   *
   * @param angle the angle in degrees
   * @return the wrapped angle in degrees within [-180, 180]
   */
  public static double wrapTo180(double angle) {
    double newAngle = (angle + 180) % 360;
    while (newAngle < 0) {
      newAngle += 360;
    }

    return newAngle - 180;
  }
}
