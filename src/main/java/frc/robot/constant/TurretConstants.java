package frc.robot.constant;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

public class TurretConstants {
  public static final int kTurretCanId = 30;

  public static final int kTurretCurrentLimit = 100;
  public static final Angle kTurretTheta = Units.Degrees.of(45.0);
  public static final MotorType kTurretMotorType = MotorType.kBrushless;

  public static final double kTurretP = 6; // 3;
  public static final double kTurretI = 0.0;
  public static final double kTurretD = 2;
  public static final double kTurretIZ = 0.0;
  public static final double kFFCommand = -0.5;
  public static final Voltage kFFBase = Units.Volts.of(0.5);

  public static final AngularVelocity kTurretMaxVelocity = Units.RadiansPerSecond.of(4.0);
  public static final AngularAcceleration kTurretMaxAcceleration = Units.RadiansPerSecondPerSecond.of(4.0);
  public static final Angle kTurretMinAngle = Units.Degrees.of(-180.0);
  public static final Angle kTurretMaxAngle = Units.Degrees.of(180.0);

  public static final int kTurretOffByMs = 20;

  // Positive = CCW (left). Live-tunable at NT /Tuning/Turret/AimTrimDeg.
  public static final double kTurretAimTrimDeg = 0.0;

  public static final boolean kMotorInverted = true;

  public static final Angle kTurretOffset = Units.Rotations.of(0.8673);
}
