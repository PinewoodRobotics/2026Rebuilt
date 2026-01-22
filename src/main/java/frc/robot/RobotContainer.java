package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.command.SwerveMoveTeleop;
import frc.robot.constant.PiConstants;
import frc.robot.hardware.AHRSGyro;
import frc.robot.subsystem.CameraSubsystem;
import frc.robot.subsystem.GlobalPosition;
import frc.robot.subsystem.OdometrySubsystem;
import frc.robot.subsystem.SwerveSubsystem;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;
import pwrup.frc.core.controller.LogitechController;
import pwrup.frc.core.controller.OperatorPanel;
import pwrup.frc.core.online.PublicationSubsystem;
import pwrup.frc.core.online.raspberrypi.PrintPiLogs;

public class RobotContainer {

  final LogitechController m_controller = new LogitechController(0);
  final OperatorPanel m_operatorPanel = new OperatorPanel(1);
  final FlightStick m_leftFlightStick = new FlightStick(2);
  final FlightStick m_rightFlightStick = new FlightStick(3);
  final FlightModule m_flightModule = new FlightModule(
      m_leftFlightStick,
      m_rightFlightStick);

  public RobotContainer() {
    CameraSubsystem.GetInstance();
    GlobalPosition.GetInstance();
    OdometrySubsystem.GetInstance();
    AHRSGyro.GetInstance();
    SwerveSubsystem.GetInstance();

    // Initialize publication subsystem for sending data to Pi
    PublicationSubsystem.GetInstance(Robot.getCommunicationClient());

    setSwerveCommands();
  }

  private void setSwerveCommands() {
    SwerveSubsystem swerveSubsystem = SwerveSubsystem.GetInstance();

    swerveSubsystem.setDefaultCommand(new SwerveMoveTeleop(swerveSubsystem, m_flightModule));

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
    var position = GlobalPosition.Get();
    if (position != null) {
      AHRSGyro.GetInstance().setAngleAdjustment(position.getRotation().getDegrees());
      OdometrySubsystem.GetInstance().setOdometryPosition(position);
      SwerveSubsystem.GetInstance().resetGyro(0);
    }

    PublicationSubsystem.addDataClasses(
        OdometrySubsystem.GetInstance(),
        AHRSGyro.GetInstance());
  }
}
