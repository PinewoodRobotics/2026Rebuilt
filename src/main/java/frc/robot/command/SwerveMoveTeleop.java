package frc.robot.command;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.pwrup.util.Vec2;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Distance;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.Command;
import frc.robot.constant.BotConstants;
import frc.robot.constant.ControllerConstants;
import frc.robot.constant.swerve.SwerveConstants;
import frc.robot.subsystem.GlobalPosition;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.SwerveSubsystem;
import frc.robot.util.AimPoint;
import frc.robot.util.LocalMath;
import lombok.Getter;
import lombok.Setter;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;

/**
 * Basic teleop swerve: joystick to velocity, gyro-relative driving.
 */
public class SwerveMoveTeleop extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final FlightModule controller;
  private final Optional<BooleanSupplier> isShootingSupplier;

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      FlightModule controller, Optional<BooleanSupplier> isShootingSupplier) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.controller = controller;
    this.isShootingSupplier = isShootingSupplier;
    addRequirements(m_swerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double multi = (BotConstants.alliance == Alliance.RED ? -1 : 1);
    double rawR = LocalMath.deadband(
        controller.leftFlightStick.getJoystickRotation(),
        ControllerConstants.kRotDeadband,
        ControllerConstants.kRotMinValue);

    double rawX = LocalMath.deadband(
        controller.rightFlightStick.getJoystickX() * multi,
        ControllerConstants.kXSpeedDeadband,
        ControllerConstants.kXSpeedMinValue);

    double rawY = LocalMath.deadband(
        controller.rightFlightStick.getJoystickY() * multi,
        ControllerConstants.kYSpeedDeadband,
        ControllerConstants.kYSpeedMinValue);

    var velocity = SwerveSubsystem.fromPercentToVelocity(
        new Vec2(rawX, rawY),
        rawR);

    if (m_swerveSubsystem.isGpsAssist() && AimPoint.getZone(GlobalPosition.Get()) == AimPoint.ZoneName.FRONT_OF_HUB
        && isShootingSupplier.isPresent() && isShootingSupplier.get().getAsBoolean()) {
      velocity = SwerveSubsystem.fromPercentToVelocity(
          new Vec2(rawX, rawY),
          rawR,
          SwerveConstants.kRobotMaxSpeed.times(SwerveConstants.kShootingSpeedMultiplier),
          SwerveConstants.kRobotMaxTurnSpeed.times(SwerveConstants.kShootingSpeedMultiplier));
    }

    m_swerveSubsystem.drive(velocity, SwerveSubsystem.DriveType.FIELD_RELATIVE);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stop();
  }
}
