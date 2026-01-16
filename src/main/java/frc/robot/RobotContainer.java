package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.command.SwerveMoveTeleop;
import frc.robot.subystem.SwerveSubsystem;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;
import pwrup.frc.core.controller.LogitechController;
import pwrup.frc.core.controller.OperatorPanel;

public class RobotContainer {

  final LogitechController m_controller = new LogitechController(0);
  final OperatorPanel m_operatorPanel = new OperatorPanel(1);
  final FlightStick m_leftFlightStick = new FlightStick(2);
  final FlightStick m_rightFlightStick = new FlightStick(3);
  final FlightModule m_flightModule = new FlightModule(
    m_leftFlightStick,
    m_rightFlightStick);
  SwerveMoveTeleop m_moveCommand;
  private Boolean isNonFieldRelative = false;

  public RobotContainer() {
    setSwerveCommands();
  }

  private void setSwerveCommands() {
    SwerveSubsystem swerveSubsystem = SwerveSubsystem.GetInstance();

    // Create and set the default teleop drive command
    this.m_moveCommand = new SwerveMoveTeleop(swerveSubsystem, m_flightModule);
    swerveSubsystem.setDefaultCommand(m_moveCommand);

    // Right Flight Stick B5: Reset gyro to 0 degrees
    m_rightFlightStick
        .B5()
        .onTrue(swerveSubsystem.runOnce(() -> {
          swerveSubsystem.resetGyro(0);
        }));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void onAnyModeStart() {
  }
}
