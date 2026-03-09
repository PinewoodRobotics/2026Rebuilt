package frc.robot.constant;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BotConstants {

  public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltWelded);

  public static final Alliance alliance = Alliance.Red; // DriverStation.getAlliance().orElse(Alliance.Blue);

  public static enum RobotVariant {
    ABOT,
    BBOT
  }

  public static final RobotVariant robotType = RobotVariant.ABOT;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Mode currentMode = Mode.REAL;
}
