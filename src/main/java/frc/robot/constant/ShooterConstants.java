package frc.robot.constant;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {
  public static final int kShooterCurrentLimit = 60;

  public static final double kShooterP = 0.00025;
  public static final double kShooterFollowerP = kShooterP; // 0.0025;
  public static final double kShooterI = 0.0;
  public static final double kShooterD = 0.0;
  public static final double kShooterIZ = 0.0;
  public static final double kFF = 0.0; // 0.0018;

  public static final boolean kShooterLeaderReversed = true;
  public static final boolean kShooterFollowerReversed = false;
  public static final double kShooterMotorRotationsPerRotation = 1.0;

  public static final int kShooterCanId = 5;
  public static final MotorType kShooterMotorType = MotorType.kBrushless;

  public static final int kShooterCanIdFollower = 6;
  public static final MotorType kShooterMotorTypeFollower = MotorType.kBrushless;
  public static final AngularVelocity kShooterMinVelocity = Units.RotationsPerSecond.of(0.0);
  public static final AngularVelocity kShooterMaxVelocity = Units.RotationsPerSecond.of(100.0);
  public static final AngularAcceleration kShooterMaxAcceleration = Units.RotationsPerSecondPerSecond.of(1000.0);

  public static final int kShooterOffByMs = 200;

  ///////////////// AIMING CONSTANTS /////////////////

  public static final double kTimeVsDistanceSlope = 0.0111;
  public static final double kTimeVsDistanceIntercept = 0.316;

  public static double DistanceFromTargetToTime(double distance) {
    return kTimeVsDistanceSlope * distance + kTimeVsDistanceIntercept;
  }
}
