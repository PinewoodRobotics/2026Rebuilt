package frc.robot.constant;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
  public static final int intakeIntakerMotorID = 2;
  public static final boolean intakeIntakerInverted = false;
  public static final double intakeMotorSpeed = 0.6;
  public static final double extakeMotorSpeed = -0.3;

  public static final int intakeWristMotorID = 11;
  public static final boolean intakeWristInverted = false;

  public static final double intakeWristP = 3;
  public static final double intakeWristI = 0.0;
  public static final double intakeWristD = 0.0;
  public static final double intakeWristIZone = 0.0;
  public final static double intakeWristFeedForwardK = 0.2;
  public final static Rotation2d intakeWristFFOffset = Rotation2d.fromRotations(0); // TODO

  public final static int intakeWristCurrentLimit = 20;
  public final static double intakeWristGearingRatio = 1.0 / 80.0;
  public final static Rotation2d intakeWristOffset = Rotation2d.fromRotations(0.862); // when the wrist is fully down
  public final static Rotation2d wristStowedAngle = Rotation2d.fromRotations(0.0);
  public final static Rotation2d wristTopAngle = Rotation2d.fromRotations(0.27);

  // public final static Rotation2d intakeWristIntakingAngle =
  // Rotation2d.fromRotations(0);

  public final static Rotation2d kTolerance = Rotation2d.fromDegrees(5);
}