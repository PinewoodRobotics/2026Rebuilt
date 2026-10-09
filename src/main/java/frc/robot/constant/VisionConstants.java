package frc.robot.constant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public record CameraConfig(String name, boolean enabled, Transform3d robotToCamera) {
  }

  public static final CameraConfig[] kCameras = {
      camera("front_left", true, 0.0836, 0.3073, 0.4795, 0.0, 45.0),
      camera("front_right", false, 0.136, -0.324, 0.0, 0.0, -52.904),
      camera("rear_left", true, -0.0836, 0.3073, 0.4795, 0.0, 135.0),
      camera("rear_right", true, -0.297, -0.297, 0.3067, 0.0, 225.0),
  };

  public static final double kMaxSingleTagDistanceM = 5.0;
  public static final double kMaxSingleTagAmbiguity = 0.2;
  public static final double kMaxPoseHeightErrorM = 0.5;

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  private static CameraConfig camera(String name, boolean enabled, double xM, double yM, double zM,
      double pitchDeg, double yawDeg) {
    // Pitch is positive-down, so a camera tilted up needs a negative pitch.
    return new CameraConfig(name, enabled, new Transform3d(
        new Translation3d(xM, yM, zM),
        new Rotation3d(0, Units.degreesToRadians(pitchDeg), Units.degreesToRadians(yawDeg))));
  }
}
