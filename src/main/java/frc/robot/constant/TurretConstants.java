package frc.robot.constant;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class TurretConstants {
  public static final int kTurretCanId = 35;
  public static final int kTurretCurrentLimit = 40;
  public static final double feedForwardFactor = 1.0;
  public static final Angle kTurretTheta = Units.Degrees.of(45.0);
  public static final double kTurretMotorRotationsPerRotation = 16.0;
  public static final MotorType kTurretMotorType = MotorType.kBrushless;

  public static final double kTurretP = 2;
  public static final double kTurretI = 0.0;
  public static final double kTurretD = 100;
  public static final double kTurretIZ = 0.0;
  public static final double kFFCommand = 7.0;

  public static final AngularVelocity kTurretMaxVelocity = Units.RadiansPerSecond.of(4.0);
  public static final AngularAcceleration kTurretMaxAcceleration = Units.RadiansPerSecondPerSecond.of(4.0);
  public static final Angle kTurretMinAngle = Units.Degrees.of(-180.0);
  public static final Angle kTurretMaxAngle = Units.Degrees.of(180.0);

  public static final int kTurretOffByMs = 200;
  public static final Translation2d turretPositionInRobot = new Translation2d(0, 1);

  public static final boolean kMotorInverted = true;

  // Total gearing from motor to turret.
  // REV MAXPlanetary 3:1 + motor gear to turret gear stage.
  public static final double kGearboxReduction = 3.0;
  public static final int kPinionTeeth = 45; // outside gear on motor output
  public static final int kTurretRingTeeth = 55; // inside gear on turret
  public static final double kGearRatio = (1.0 / kGearboxReduction)
      * ((double) kPinionTeeth / (double) kTurretRingTeeth);

  /*
   * 
   * public static class TurretConstants {
   * public static final int kTurretMotorID = 35;
   * public static final boolean kMotorInverted = false;
   * 
   * // Very conservative startup limits for safe bring-up and hand tuning.
   * public static final int kCurrentLimitA = 10;
   * public static final double kOpenLoopRampSeconds = 3.0;
   * public static final double kClosedLoopRampSeconds = 3.0;
   * // public static final double kMinOutput = -0.08;
   * // public static final double kMaxOutput = 0.08;
   * 
   * public static final double kP = 0.005;
   * public static final double kI = 0.0;
   * public static final double kD = 0.5;
   * 
   * // Total gearing from motor to turret.
   * // REV MAXPlanetary 3:1 + motor gear to turret gear stage.
   * public static final double kGearboxReduction = 3.0;
   * public static final int kPinionTeeth = 45; // outside gear on motor output
   * public static final int kTurretRingTeeth = 55; // inside gear on turret
   * public static final boolean kExternalMeshInvertsDirection = true;
   * 
   * public static final double kTurretRotationsPerMotorRotation =
   * (1.0 / kGearboxReduction)
   * ((double) kPinionTeeth / (double) kTurretRingTeeth)
   * (kExternalMeshInvertsDirection ? -1.0 : 1.0);
   * // REV primary encoder conversion factors must be positive.
   * public static final double kTurretDegreesPerMotorRotation = 360.0 *
   * Math.abs(kTurretRotationsPerMotorRotation);
   * public static final double kTurretDirectionSign =
   * kExternalMeshInvertsDirection ? -1.0 : 1.0;
   * 
   * public static final Angle kHomeAngle = Units.Degrees.of(0.0);
   * // Allowed turret command window in field-relative degrees.
   * // Set to +/-180 so H2 can command the full direction circle.
   * public static final Angle kMinAngle = Units.Degrees.of(-180.0);
   * public static final Angle kMaxAngle = Units.Degrees.of(180.0);
   * public static final Angle kTolerance = Units.Degrees.of(3.0);
   * public static final Angle kSmoothWindow = Units.Degrees.of(20.0);
   * public static final Angle kSmoothMaxStepPerCycle = Units.Degrees.of(2.0);
   * }
   */
}
