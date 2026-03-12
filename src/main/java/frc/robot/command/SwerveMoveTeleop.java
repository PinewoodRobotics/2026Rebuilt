package frc.robot.command;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.pwrup.util.Vec2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.ControllerConstants;
import frc.robot.subsystem.GlobalPosition;
import frc.robot.subsystem.SwerveSubsystem;
import frc.robot.util.LocalMath;
import lombok.Getter;
import lombok.Setter;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;

/**
 * Basic teleop swerve: joystick to velocity, gyro-relative driving.
 */
public class SwerveMoveTeleop extends Command {

  public enum AxisConstraint {
    X,
    Y,
    XY;

    public Translation2d getConversion() {
      return switch (this) {
        case X -> new Translation2d(1, 0);
        case Y -> new Translation2d(0, 1);
        case XY -> new Translation2d(1, 1);
      };
    }
  }

  /**
   * Lane segment: pose = center, length = total length (extends length/2 each
   * way).
   */
  public record Lane(Pose2d pose, double length, AxisConstraint axisConstant) {
    public void log() {
      Translation2d direction = new Translation2d(1, 0).rotateBy(pose.getRotation());
      Translation2d half = direction.times(length / 2);
      Pose2d startPose = new Pose2d(pose.getTranslation().minus(half), pose.getRotation());
      Pose2d endPose = new Pose2d(pose.getTranslation().plus(half), pose.getRotation());
      Logger.recordOutput("SwerveMoveTeleop/Lane/Endpoints", new Pose2d[] { startPose, endPose });
      Logger.recordOutput("SwerveMoveTeleop/Lane/Length", length);
    }

    public Pose2d nearestPoint(Pose2d otherPose) {
      Pose2d lanePose = this.pose();
      double laneLength = this.length();

      Translation2d dir = new Translation2d(1, 0).rotateBy(lanePose.getRotation());
      Translation2d a = lanePose.getTranslation().minus(dir.times(laneLength / 2));
      Translation2d ab = dir.times(laneLength);
      Translation2d p = otherPose.getTranslation();
      Translation2d ap = p.minus(a);
      double ab2 = ab.getX() * ab.getX() + ab.getY() * ab.getY();
      if (ab2 < 1e-9) {
        return new Pose2d(a, lanePose.getRotation());
      }
      double t = (ap.getX() * ab.getX() + ap.getY() * ab.getY()) / ab2;
      t = Math.max(0, Math.min(1, t));
      Translation2d closest = a.plus(ab.times(t));
      return new Pose2d(closest, lanePose.getRotation());
    }
  }

  private final SwerveSubsystem m_swerveSubsystem;
  private final FlightModule controller;

  private final List<Lane> lanes;

  private final PIDController lanePullPid;
  private ChassisSpeeds filteredPercent = new ChassisSpeeds();

  private static final double kLanePullMaxSpeed = 0.4;
  private static final Distance minDistance = Distance.ofRelativeUnits(0.8, Units.Meters);

  private static final double kNearLaneJoystickAlpha = 0.2;
  private static final double kFarLaneJoystickAlpha = 0.6;

  private static final double kMaxPerpendicularScale = 1.0;
  private static final double kMinPerpendicularScale = 0.2;

  private Supplier<Boolean> areGpsFeaturesEnabled;

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      FlightModule controller) {
    this(swerveSubsystem, controller, new ArrayList<>(), () -> true);
  }

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      FlightModule controller,
      Lane[] lanes, Supplier<Boolean> shouldAdjustVelocity) {
    this(swerveSubsystem, controller, Arrays.asList(lanes), shouldAdjustVelocity);
  }

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      FlightModule controller,
      List<Lane> lanes, Supplier<Boolean> shouldAdjustVelocity) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.controller = controller;
    this.lanes = lanes;
    this.lanePullPid = new PIDController(0.5, 0, 0);
    this.areGpsFeaturesEnabled = shouldAdjustVelocity;
    addRequirements(m_swerveSubsystem);
  }

  @Override
  public void execute() {
    double rawR = LocalMath.deadband(
        controller.leftFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKROTATION.value),
        ControllerConstants.kRotDeadband,
        ControllerConstants.kRotMinValue);

    double rawX = LocalMath.deadband(
        controller.rightFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKY.value),
        ControllerConstants.kXSpeedDeadband,
        ControllerConstants.kXSpeedMinValue);

    double rawY = LocalMath.deadband(
        controller.rightFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKX.value),
        ControllerConstants.kYSpeedDeadband,
        ControllerConstants.kYSpeedMinValue);

    var velocity = SwerveSubsystem.fromPercentToVelocity(
        new Vec2(rawX, rawY),
        rawR);

    if (areGpsFeaturesEnabled.get()) {
      velocity = applyGpsFeatures(velocity);
    }

    m_swerveSubsystem.drive(velocity, SwerveSubsystem.DriveType.FIELD_RELATIVE);
  }

  private ChassisSpeeds applyGpsFeatures(ChassisSpeeds rawVelocityInput) {
    rawVelocityInput = adjustVelocityForLane(rawVelocityInput);
    return rawVelocityInput;
  }

  private ChassisSpeeds adjustVelocityForLane(ChassisSpeeds rawVelocityInput) {
    Pose2d currentPose = GlobalPosition.Get();
    if (currentPose == null) {
      return rawVelocityInput;
    }

    Lane nearestLane = getNearestLanePoint(currentPose, minDistance);
    if (nearestLane == null) {
      return rawVelocityInput;
    }

    Pose2d nearestLanePoint = nearestLane.nearestPoint(currentPose);
    double distanceRatio = MathUtil.clamp(
        currentPose.getTranslation().getDistance(nearestLanePoint.getTranslation()) / minDistance.in(Units.Meters),
        0,
        1);

    rawVelocityInput = smoothVelocity(rawVelocityInput.vxMetersPerSecond, rawVelocityInput.vyMetersPerSecond,
        rawVelocityInput.omegaRadiansPerSecond, distanceRatio);

    Logger.recordOutput("SwerveMoveTeleop/NearestLanePoint", nearestLanePoint);
    nearestLane.log();

    Translation2d lanePullVelocity = getLanePull(nearestLanePoint, currentPose, nearestLane.axisConstant());
    Translation2d laneDriveScale = getLaneDriveScale(distanceRatio, nearestLane.axisConstant());

    rawVelocityInput.vxMetersPerSecond *= laneDriveScale.getX() == 0 ? 1.0 : laneDriveScale.getX();
    rawVelocityInput.vyMetersPerSecond *= laneDriveScale.getY() == 0 ? 1.0 : laneDriveScale.getY();

    rawVelocityInput.vxMetersPerSecond += lanePullVelocity.getX();
    rawVelocityInput.vyMetersPerSecond += lanePullVelocity.getY();

    return rawVelocityInput;
  }

  private ChassisSpeeds smoothVelocity(double x, double y, double r, double distanceRatio) {
    double laneSmoothingAlpha = MathUtil.interpolate(kNearLaneJoystickAlpha, kFarLaneJoystickAlpha, distanceRatio);

    filteredPercent.vxMetersPerSecond = MathUtil.interpolate(filteredPercent.vxMetersPerSecond, x, laneSmoothingAlpha);
    filteredPercent.vyMetersPerSecond = MathUtil.interpolate(filteredPercent.vyMetersPerSecond, y, laneSmoothingAlpha);
    filteredPercent.omegaRadiansPerSecond = MathUtil.interpolate(filteredPercent.omegaRadiansPerSecond, r,
        laneSmoothingAlpha);

    return new ChassisSpeeds(
        filteredPercent.vxMetersPerSecond,
        filteredPercent.vyMetersPerSecond,
        filteredPercent.omegaRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stop();
  }

  /**
   * Returns a velocity vector (m/s) that pulls the robot toward the nearest lane
   * point, independent of lane orientation.
   */
  private Translation2d getLanePull(Pose2d nearestLanePoint, Pose2d relativeTo, AxisConstraint axisConstraint) {
    Translation2d error = nearestLanePoint.getTranslation().minus(relativeTo.getTranslation());
    double errorDistance = error.getNorm();
    if (errorDistance < 1e-6) {
      return new Translation2d();
    }

    double pullSpeed = MathUtil.clamp(
        Math.abs(lanePullPid.calculate(errorDistance, 0)),
        0,
        kLanePullMaxSpeed);
    Translation2d pullDirection = error.div(errorDistance);

    Translation2d conversionFactor = axisConstraint.getConversion();
    pullDirection = pullDirection.times(pullSpeed);

    return new Translation2d(
        pullDirection.getX() * conversionFactor.getX(),
        pullDirection.getY() * conversionFactor.getY());
  }

  private Translation2d getLaneDriveScale(double distanceRatio, AxisConstraint axisConstraint) {
    double constrainedScale = MathUtil.interpolate(kMinPerpendicularScale, kMaxPerpendicularScale, distanceRatio);
    Translation2d conversionFactor = axisConstraint.getConversion();
    return new Translation2d(
        constrainedScale * conversionFactor.getX(),
        constrainedScale * conversionFactor.getY());
  }

  public Lane getNearestLanePoint(Pose2d relativeTo, Distance distanceLimit) {
    if (relativeTo == null) {
      return null;
    }

    Lane nearestLane = null;
    double nearestDistance = Double.POSITIVE_INFINITY;
    double maxDistanceMeters = distanceLimit.in(Units.Meters);

    for (Lane lane : lanes) {
      Pose2d lanePoint = lane.nearestPoint(relativeTo);
      double laneDistance = lanePoint.getTranslation().getDistance(relativeTo.getTranslation());
      if (laneDistance < maxDistanceMeters) {
        if (laneDistance < nearestDistance) {
          nearestLane = lane;
          nearestDistance = laneDistance;
        }
      }
    }

    return nearestLane;
  }
}
