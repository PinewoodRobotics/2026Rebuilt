package frc.robot.constant;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldConstants {
  public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltWelded);

  public static final Translation3d kHubPositionRed = new Translation3d(12, 4, 0);
}
