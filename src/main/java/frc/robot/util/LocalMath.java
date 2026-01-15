package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * @note MathFun = Math Functions
 * @apiNote this is the file where all of the math functions go
 */
public class LocalMath {

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

  public static Translation2d fromGlobalToRelative(Translation2d globalRobotPose, Translation2d globalTargetPose) {
    return globalTargetPose.minus(globalRobotPose);
  }
}
