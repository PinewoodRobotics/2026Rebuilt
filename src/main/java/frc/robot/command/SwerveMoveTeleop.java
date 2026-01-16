package frc.robot.command;

import org.pwrup.util.Vec2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.swerve.SwerveConstants;
import frc.robot.subystem.SwerveSubsystem;
import frc.robot.util.CustomMath;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;

/**
 * Basic teleop swerve: joystick to velocity, gyro-relative driving.
 */
public class SwerveMoveTeleop extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final FlightModule controller;

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      FlightModule controller) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.controller = controller;
    addRequirements(m_swerveSubsystem);
  }

  @Override
  public void execute() {
    final var c = SwerveConstants.INSTANCE;
    double r = CustomMath.deadband(
        controller.leftFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKROTATION.value) * -1,
        c.kRotDeadband,
        c.kRotMinValue);

    double x = CustomMath.deadband(
        controller.rightFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKY.value) * -1,
        c.kXSpeedDeadband,
        c.kXSpeedMinValue);

    double y = CustomMath.deadband(
        controller.rightFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKX.value) * -1,
        c.kYSpeedDeadband,
        c.kYSpeedMinValue);

    var velocity = SwerveSubsystem.fromPercentToVelocity(new Vec2(x, y), r);

    m_swerveSubsystem.drive(velocity, SwerveSubsystem.DriveType.GYRO_RELATIVE);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stop();
  }
}
