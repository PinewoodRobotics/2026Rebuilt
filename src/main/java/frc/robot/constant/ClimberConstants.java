package frc.robot.constant;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class ClimberConstants {

  public static enum ClimberPosition {
    L1(new Distance[] { Units.Meters.of(0.0), Units.Meters.of(1.0) }),
    L2(new Distance[] { Units.Meters.of(1.0), Units.Meters.of(0.0) });

    public final Distance[] positionMoveSequence;

    ClimberPosition(Distance[] positionMoveSequence) {
      this.positionMoveSequence = positionMoveSequence;
    }
  }

  public final static double kGearRatio = 0.0;
  public final static double kAxleToHeightRatio = 0.0;
  public final static double kGearHeightRatio = kGearRatio * kAxleToHeightRatio;

  public static final int kLeftMotorID = 8;
  public static final int kRightMotorID = 9;

  public final static double kP = 0.6;
  public final static double kI = 0.08;
  public final static double kD = 0;
  public final static double kIZone = Double.POSITIVE_INFINITY;
  public final static double kDifSpeedMultiplier = 0;
  public final static double kS = 0;
  public final static double kV = 0;
  public final static double kG = 0.02;
  public final static double kA = 0;
  public final static double kTolerance = 0.1;

  public final static boolean kSetpointRamping = true;
  public final static double kMaxSetpointRamp = 0.15;

  public static final MotorType kMotorType = MotorType.kBrushless;

  public static final boolean kLeftMotorInverted = true;
  public static final boolean kRightMotorInverted = false;

  public static final Distance kMaxHeight = Units.Meters.of(10.0);
  public static final Distance kMinHeight = Units.Meters.of(0.0);

  public static final Distance kStartingHeight = Units.Meters.of(0.0);
}
