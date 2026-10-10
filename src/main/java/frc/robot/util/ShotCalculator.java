package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constant.ShooterConstants;
import frc.robot.constant.TurretConstants;

public final class ShotCalculator {
  public record ShotSolution(
      Translation2d targetFromTurret,
      double rawDistance,
      Translation2d compensatedTargetFromTurret,
      Translation2d leadCompensation,
      double compensatedDistance,
      double flightTime,
      AngularVelocity shooterVelocity,
      double turretAngle,
      Translation2d turretFieldVelocity,
      boolean distanceClamped,
      boolean reachable) {
    public boolean isValid() {
      return isFinite() && reachable;
    }

    public boolean isFinite() {
      return Double.isFinite(turretAngle)
          && Double.isFinite(compensatedDistance)
          && Double.isFinite(flightTime)
          && Double.isFinite(shooterVelocity.in(Units.RPM));
    }
  }

  private ShotCalculator() {
  }

  public static ShotSolution Calculate(
      Pose2d robotPose,
      Translation2d targetGlobal,
      ChassisSpeeds robotFieldSpeeds) {
    return Calculate(robotPose, targetGlobal, robotFieldSpeeds, TurretConstants.kTurretPositionInRobot);
  }

  public static ShotSolution Calculate(
      Pose2d robotPose,
      Translation2d targetGlobal,
      ChassisSpeeds robotFieldSpeeds,
      Translation2d turretPositionInRobot) {
    Rotation2d heading = robotPose.getRotation();
    Translation2d turretOffsetField = turretPositionInRobot.rotateBy(heading);
    Translation2d turretField = robotPose.getTranslation().plus(turretOffsetField);
    Translation2d turretFieldVelocity = GetTurretFieldVelocity(robotFieldSpeeds, turretOffsetField);

    Translation2d targetFromTurretField = targetGlobal.minus(turretField);
    double leadTime = SolveLeadTime(targetFromTurretField, turretFieldVelocity);
    Translation2d aimFromTurretField = targetFromTurretField.minus(turretFieldVelocity.times(leadTime));

    Rotation2d fieldToRobot = heading.unaryMinus();
    Translation2d targetFromTurret = targetFromTurretField.rotateBy(fieldToRobot);
    Translation2d compensatedTargetFromTurret = aimFromTurretField.rotateBy(fieldToRobot);
    double compensatedDistance = compensatedTargetFromTurret.getNorm();
    double flightTime = ShooterConstants.DistanceFromTargetToTime(compensatedDistance);

    return new ShotSolution(
        targetFromTurret,
        targetFromTurret.getNorm(),
        compensatedTargetFromTurret,
        compensatedTargetFromTurret.minus(targetFromTurret),
        compensatedDistance,
        flightTime,
        ShooterConstants.DistanceFromTargetToVelocity(compensatedDistance),
        Math.atan2(compensatedTargetFromTurret.getY(), compensatedTargetFromTurret.getX()),
        turretFieldVelocity,
        !ShooterConstants.IsWithinCalibratedDistance(compensatedDistance),
        IsBallClosingOnTarget(targetFromTurretField, aimFromTurretField, turretFieldVelocity, flightTime));
  }

  // The shooter can only throw as far as the calibrated range in the fit's flight time, so a
  // turret receding faster than that average ball speed launches a ball that never closes on
  // the target. A slow or stationary robot beyond the range still closes and stays valid.
  private static boolean IsBallClosingOnTarget(
      Translation2d targetFromTurretField,
      Translation2d aimFromTurretField,
      Translation2d turretFieldVelocity,
      double flightTime) {
    double aimDistance = aimFromTurretField.getNorm();
    Translation2d ballVelocityFromTurret = Translation2d.kZero;
    if (aimDistance > 0.0) {
      double ballSpeed = ShooterConstants.ClampToCalibratedDistance(aimDistance) / flightTime;
      ballVelocityFromTurret = aimFromTurretField.times(ballSpeed / aimDistance);
    }
    return ballVelocityFromTurret.plus(turretFieldVelocity).dot(targetFromTurretField) > 0.0;
  }

  // Solves t = flightTime(|target - v * t|) by bisection. Fixed-point iteration on that
  // equation diverges once the turret moves faster than 1 / kTimeVsDistanceSlope (~3.4 m/s),
  // which the drivetrain can exceed. The flight-time fit is clamped to the calibrated
  // distance range, so its output range always brackets a root.
  private static double SolveLeadTime(Translation2d targetFromTurretField, Translation2d turretFieldVelocity) {
    double low = ShooterConstants.DistanceFromTargetToTime(ShooterConstants.kMinCalibratedDistanceMeters);
    double high = ShooterConstants.DistanceFromTargetToTime(ShooterConstants.kMaxCalibratedDistanceMeters);
    while (high - low > ShooterConstants.kLeadFlightTimeToleranceSeconds) {
      double guess = (low + high) / 2.0;
      Translation2d aim = targetFromTurretField.minus(turretFieldVelocity.times(guess));
      if (ShooterConstants.DistanceFromTargetToTime(aim.getNorm()) > guess) {
        low = guess;
      } else {
        high = guess;
      }
    }
    return (low + high) / 2.0;
  }

  private static Translation2d GetTurretFieldVelocity(ChassisSpeeds robotFieldSpeeds, Translation2d turretOffsetField) {
    double omega = robotFieldSpeeds.omegaRadiansPerSecond;
    return new Translation2d(
        robotFieldSpeeds.vxMetersPerSecond - omega * turretOffsetField.getY(),
        robotFieldSpeeds.vyMetersPerSecond + omega * turretOffsetField.getX());
  }
}
