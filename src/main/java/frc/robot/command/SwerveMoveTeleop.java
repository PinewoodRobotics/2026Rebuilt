package frc.robot.command;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
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
    double multi = (BotConstants.alliance == Alliance.Red ? -1 : 1);
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
