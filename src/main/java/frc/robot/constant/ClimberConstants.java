package frc.robot.constant;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class ClimberConstants {
  public static final Distance kMaxHeight = Units.Inches.of(12.4);
  public static final Distance kMinHeight = Units.Inches.of(0.0);
  public static final Distance kStartingHeight = Units.Inches.of(0.0);

  public static final double kGearRatio = 1.0 / 75.0;
  public static final Distance kAxleToHeightRatio = Units.Inches.of(5.5);

  public final static double kGearHeightRatio = kGearRatio * kAxleToHeightRatio.in(Units.Meters);

  public static final int kClimberMotorID = 46;
  public static final boolean kMotorInverted = true;

  public final static double kP = 10;
  public final static double kI = 0;
  public final static double kD = 0;
  public final static double kIZone = 0;
  public final static double kS = 0;
  public final static double kV = 0;
  public final static double kG = 0;
  public final static double kA = 0;
  public final static double kVelocityP = 0;
  public final static double kVelocityI = 0;
  public final static double kVelocityD = 0;
  public final static double kTolerance = Units.Feet.of(0.1).in(Units.Meters);

  public final static boolean kSetpointRamping = true;
  public static final double kMaxSetpointRamp = Units.Feet.of(0.15).in(Units.Meters);
  public static final double kOpenLoopRampSeconds = 0.25;
  public static final double kClosedLoopRampSeconds = 0.25;
  public static final LinearVelocity kManualDownVelocity = Units.MetersPerSecond.of(-0.10);
  public static final LinearVelocity kCalibrationVelocity = Units.MetersPerSecond.of(-0.10);
  public static final double kCalibrationVelocityToleranceMetersPerSecond = 0.01;
  public static final double kCalibrationMinRuntimeSeconds = 0.25;
  public static final double kCalibrationSettledTimeSeconds = 0.20;

  public static final MotorType kMotorType = MotorType.kBrushless;
  public static final int kLiftCurrentLimit = 40;
}
