package frc.robot.constant;

import java.util.Optional;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ClimberConstants {

  private static final Distance highest = Units.Inches.of(12.4);
  private static final Distance lowest = Units.Inches.of(0.0);

  public static final double kGearRatio = 1.0 / 15.0;
  public static final double kAxleToHeightRatio = Units.Inches.of(5.5).in(Units.Meters); // just recalculated to 5.5in
                                                                                         // actually!
  public final static double kGearHeightRatio = kGearRatio * kAxleToHeightRatio;

  public static final int kLeftMotorID = 46;
  public static final int kWristMotorID = 47;

  public final static double kP = 10;
  public final static double kI = 0;
  public final static double kD = 0;
  public final static double kWristP = 0.1;
  public final static double kWristI = 0;
  public final static double kWristD = 0;
  // public final static double kIZone = Double.POSITIVE_INFINITY;
  public final static double kIZone = 0;
  public final static double kWristIZone = 0;
  public final static double kDifSpeedMultiplier = 0;
  public final static double kS = 0;
  public final static double kV = 0;
  public final static double kG = 0;
  public final static double kA = 0;
  public final static double kWristS = 0;
  public final static double kWristV = 0;
  public final static double kWristG = 0;
  public final static double kWristA = 0;
  public static final double kWristHoldMaxPower = 0.15;
  public static final double kWristTolerance = 0.02;
  public static final double kTolerance = Units.Feet.of(0.1).in(Units.Meters);

  public final static boolean kSetpointRamping = true;
  public static final double kMaxSetpointRamp = Units.Feet.of(0.15).in(Units.Meters);

  public static final MotorType kMotorType = MotorType.kBrushless;
  public static final int kLiftCurrentLimit = 30;

  public static final double kWristGearRatio = 1.0 / 15.0;
  public static final boolean kLeftMotorInverted = true;
  public static final boolean kRightMotorInverted = true;
  public static final boolean kWristMotorInverted = true;
  public static final int kWristCurrentLimit = 20;
  public static final double kWristEngagePower = 0.18;
  public static final double kWristDisengagePower = -0.15;
  public static final double kWristTestPower = 0.12;

  public static final Distance kMaxHeight = Units.Feet.of(2.0);
  public static final Distance kMinHeight = Units.Meters.of(0.0);

  public static final Distance kStartingHeight = Units.Meters.of(0.0);
}
