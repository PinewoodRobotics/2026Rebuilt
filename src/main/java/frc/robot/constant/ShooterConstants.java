package frc.robot.constant;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {
  public static final int kShooterCurrentLimit = 60;

  public static final double kShooterP = 0.0002;
  public static final double kShooterFollowerP = kShooterP; // 0.0025;
  public static final double kShooterI = 0.000000005;
  public static final double kShooterD = 0.02;
  public static final double kShooterIZ = 0.0;
  public static final double kFFLeader = 0.00183;
  public static final double kFFFollower = 0.0018;

  public static final boolean kShooterLeaderReversed = true;
  public static final boolean kShooterFollowerReversed = false;
  public static final double kShooterMotorRotationsPerRotation = 1.0;

  public static final int kShooterCanId = 31;
  public static final MotorType kShooterMotorType = MotorType.kBrushless;

  public static final int kShooterCanIdFollower = 32;
  public static final MotorType kShooterMotorTypeFollower = MotorType.kBrushless;
  public static final AngularVelocity kShooterMinVelocity = Units.RotationsPerSecond.of(25);
  public static final AngularVelocity kShooterMaxVelocity = Units.RotationsPerSecond.of(50.0);
  public static final AngularVelocity kShooterVelocityTolerance = Units.RotationsPerSecond.of(1);

  public static final AngularVelocity kShooterBaseSpeed = Units.RotationsPerSecond.of(25.0);

  ///////////////// AIMING CONSTANTS /////////////////

  public static final double kTimeVsDistanceSlope = 0.295;
  public static final double kTimeVsDistanceIntercept = 0.433;

  public static final double kRPMVsDistanceSlope = 135;
  public static final double kRPMVsDistanceIntercept = 1928;

  public static final double kOutMult = 1.0;

  public static double DistanceFromTargetToTime(double distance) {
    return kTimeVsDistanceSlope * distance + kTimeVsDistanceIntercept;
  }

  public static AngularVelocity DistanceFromTargetToVelocity(double distance) {
    double rpm = kRPMVsDistanceSlope * distance + kRPMVsDistanceIntercept;
    return Units.RotationsPerSecond.of((rpm * kOutMult / 60));
  }
}
