package frc.robot.constant;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
  public final static String intakeType = "BELT";
  public static final int intakeIntakerMotorID = (intakeType == "BELT" ? 46 : 33);
  public static final int intakeWristMotorID = (intakeType == "BELT" ? 43 : 34);
  public static final int intakeWristFollowerMotorID = 44;

  public static final boolean intakeIntakerInverted = (intakeType == "BELT" ? false : true);
  public static final double intakeMotorSpeed = (intakeType == "BELT" ? 0.5 : 0.15);
  /**
   * Dashboard override topic under the shared table; a request outside [0, 1]
   * means "use intakeMotorSpeed".
   */
  public static final String kIntakeSpeedTopic = "Intake/Speed";
  public static final double extakeMotorSpeed = (intakeType == "BELT" ? -0.5 : -0.2);

  public static final boolean intakeWristInverted = (intakeType == "BELT" ? true : false);

  public static final double intakeWristP = 3;
  public static final double intakeWristI = 0.0;
  public static final double intakeWristD = 0.0;
  public static final double intakeWristIZone = 0.0;
  public final static double intakeWristFeedForwardK = 0.2;
  public final static Rotation2d intakeWristFFOffset = Rotation2d.fromRotations(0); // TODO

  public final static int intakeWristCurrentLimit = 20;
  public final static double intakeWristGearingRatio = 1.0 / 80.0;
  public final static Rotation2d intakeWristOffset = Rotation2d.fromRotations(intakeType == "BELT" ? 0.84 : 0.113);
  public final static boolean intakeWristEncoderInverted = (intakeType == "BELT" ? false : true);
  public final static boolean kShouldWeAreWeGoingToPulseTheIntakeConstantVariable = false;

  public static enum WristRaiseLocation {
    BOTTOM(Rotation2d.fromRotations(0)),
    MIDDLE(Rotation2d.fromRotations(intakeType == "BELT" ? 0.338 : 0.1)),
    TOP(Rotation2d.fromRotations(0.338));

    public final Rotation2d position;

    WristRaiseLocation(Rotation2d position) {
      this.position = position;
    }
  }

  // public final static Rotation2d intakeWristIntakingAngle =
  // Rotation2d.fromRotations(0);

  public final static Rotation2d kTolerance = Rotation2d.fromDegrees(5);

  public final static double kWristDropSpeed = -0.2;
  /**
   * Stowed reads ~0.548 and the reading rises through the release point toward
   * the ground at 1.0, where it wraps to 0. Readings below the wrap threshold
   * mean the wrist already passed the ground.
   */
  public final static double kWristDropReleasePosition = 0.667;
  public final static double kWristDropWrapThreshold = 0.3;
  public final static double kWristDropTimeoutS = 2.0;
}