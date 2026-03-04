package frc.robot.command;

import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
import frc.robot.constant.swerve.SwerveConstants;
import frc.robot.subsystem.GlobalPosition;
import frc.robot.subsystem.SwerveSubsystem;
import frc.robot.util.LocalMath;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;

/**
 * Basic teleop swerve: joystick to velocity, gyro-relative driving.
 */
public class SwerveMoveTeleop extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final FlightModule controller;
  private final HashMap<Pose2d, Double> lanes;
  /** Max magnitude (m/s) of lane Y correction added to vy. */
  private static final double kLaneYVelocityGain = 0.6;
  private static final Distance minDistance = Distance.ofRelativeUnits(0.4, Units.Meters);
  private final PIDController laneYVelocityPIDController;

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      FlightModule controller) {
    this(swerveSubsystem, controller, new HashMap<>());
  }

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      FlightModule controller,
      HashMap<Pose2d, Double> lanes) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.controller = controller;
    this.lanes = lanes;
    laneYVelocityPIDController = new PIDController(1, 0, 0);

    addRequirements(m_swerveSubsystem);
  }

  @Override
  public void execute() {
    double r = LocalMath.deadband(
        controller.leftFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKROTATION.value) * -1,
        ControllerConstants.kRotDeadband,
        ControllerConstants.kRotMinValue);

    double x = LocalMath.deadband(
        controller.rightFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKY.value),
        ControllerConstants.kXSpeedDeadband,
        ControllerConstants.kXSpeedMinValue);

    double y = LocalMath.deadband(
        controller.rightFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKX.value),
        ControllerConstants.kYSpeedDeadband,
        ControllerConstants.kYSpeedMinValue);

    var velocity = SwerveSubsystem.fromPercentToVelocity(new Vec2(x, y), r);

    Lane nearestLane = null;
    if (!lanes.isEmpty()
        && (nearestLane = getNearestLane(GlobalPosition.Get(),
            minDistance)) != null) {
      var nearestLanePoint = getNearestLanePoint(GlobalPosition.Get(), nearestLane);

      double addedYVelocity = getAddedYVelocity(nearestLanePoint, GlobalPosition.Get(), velocity);
      velocity.vyMetersPerSecond -= addedYVelocity;

      Logger.recordOutput("SwerveMoveTeleop/AddedYVelocity", addedYVelocity);
      nearestLane.log();
      Logger.recordOutput("SwerveMoveTeleop/NearestLanePoint", nearestLanePoint);
    } else {
      laneYVelocityPIDController.reset();
    }

    Logger.recordOutput("SwerveMoveTeleop/Velocity", velocity);

    m_swerveSubsystem.drive(velocity, SwerveSubsystem.DriveType.GYRO_RELATIVE);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stop();
  }

  /**
   * Returns a velocity (m/s) to add to vy. Velocity fed into swerve is
   * field-relative.
   * Uses PID on field Y error (nearest lane point minus robot).
   */
  private double getAddedYVelocity(Pose2d nearestLanePoint, Pose2d relativeTo, ChassisSpeeds velocityUserInput) {
    double errorY = nearestLanePoint.getTranslation().getY() - relativeTo.getTranslation().getY();
    double output = laneYVelocityPIDController.calculate(errorY, 0);
    return MathUtil.clamp(output, -kLaneYVelocityGain, kLaneYVelocityGain);
  }

  private record Lane(Pose2d pose, double length) {
    public void log() {
      Pose2d frontPose = pose;
      Translation2d direction = new Translation2d(1, 0).rotateBy(pose.getRotation());
      Translation2d endTranslation = pose.getTranslation().plus(direction.times(length));
      Pose2d endPose = new Pose2d(endTranslation, pose.getRotation());
      Pose2d[] endpoints = new Pose2d[] { frontPose, endPose };

      Logger.recordOutput("SwerveMoveTeleop/Lane/Endpoints", endpoints);
      Logger.recordOutput("SwerveMoveTeleop/Lane/Length", length);
    }
  }

  public Lane getNearestLane(Pose2d relativeTo, Distance distanceLimit) {
    return lanes.entrySet().stream()
        .min(Comparator.comparingDouble(entry -> entry.getKey().getY() - relativeTo.getY()))
        .map(entry -> new Lane(entry.getKey(), entry.getValue()))
        .filter(lane -> lane.pose.getY() - relativeTo.getY() < distanceLimit.in(Units.Meters))
        .orElse(null);
  }

  /**
   * Returns the closest point on the lane segment to the given pose.
   * Lane is a zero-width segment: start = relativeTo position, direction =
   * relativeTo rotation (+X),
   * length = laneLength.
   */
  public Pose2d getNearestLanePoint(Pose2d pose, Lane lane) {
    Pose2d lanePose = lane.pose();
    double laneLength = lane.length();

    Translation2d a = lanePose.getTranslation();
    Translation2d dir = new Translation2d(1, 0).rotateBy(lanePose.getRotation());
    Translation2d ab = dir.times(laneLength);
    Translation2d p = pose.getTranslation();
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
