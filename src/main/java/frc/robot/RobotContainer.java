package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;

// import frc.robot.command.SwerveMoveTeleop;
// import frc.robot.command.scoring.ContinuousAimCommand;
// import frc.robot.constant.BotConstants;
// import frc.robot.constant.FieldConstants;
// import frc.robot.hardware.AHRSGyro;
// import frc.robot.subsystem.CameraSubsystem;
// import frc.robot.subsystem.GlobalPosition;
// import frc.robot.subsystem.OdometrySubsystem;
// import frc.robot.subsystem.SwerveSubsystem;
// import frc.robot.subsystem.TurretSubsystem;
// import frc.robot.util.AimPoint;
// import frc.robot.util.PathPlannerSetup;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;
import pwrup.frc.core.controller.OperatorPanel;
// import pwrup.frc.core.online.PublicationSubsystem;

import frc.robot.subsystem.IndexSubsystem;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.OrchestraSubsystem;
import frc.robot.command.testing.IndexCommand;
import frc.robot.command.testing.IntakeCommand;
import frc.robot.command.testing.SetWristPos;

public class RobotContainer {
  final OperatorPanel m_operatorPanel = new OperatorPanel(1);
  final FlightStick m_leftFlightStick = new FlightStick(2);
  final FlightStick m_rightFlightStick = new FlightStick(3);
  final FlightModule m_flightModule = new FlightModule(
      m_leftFlightStick,
      m_rightFlightStick);

  public RobotContainer() {
    // GlobalPosition.GetInstance();
    // OdometrySubsystem.GetInstance();
    // AHRSGyro.GetInstance();
    // SwerveSubsystem.GetInstance();
    // CameraSubsystem.GetInstance();
    // TurretSubsystem.GetInstance();
    IndexSubsystem.GetInstance();
    IntakeSubsystem.GetInstance();
    OrchestraSubsystem.GetInstance();

    // Initialize publication subsystem for sending data to Pi
    // PublicationSubsystem.GetInstance(Robot.getCommunicationClient());
    // PathPlannerSetup.configure();

    // setSwerveCommands();
    // setTurretCommands();
    setTestCommands();
  }

  private void setTestCommands() {
    IndexSubsystem indexSubsystem = IndexSubsystem.GetInstance();
    IntakeSubsystem intakeSubsystem = IntakeSubsystem.GetInstance();
    OrchestraSubsystem orchestraSubsystem = OrchestraSubsystem.GetInstance();
    m_leftFlightStick
        .B17()
        .whileTrue(new IndexCommand(indexSubsystem, 0.45));
    m_rightFlightStick
        .B16()
        .whileTrue(new IntakeCommand(intakeSubsystem, 0.60));
    m_rightFlightStick
        .B17()
        .whileTrue(new IntakeCommand(intakeSubsystem, -0.30));
    m_leftFlightStick
        .B7()
        .onTrue(new SetWristPos(intakeSubsystem, Rotation2d.fromRotations(0.245)));
    m_leftFlightStick
        .B8()
        .onTrue(new SetWristPos(intakeSubsystem, Rotation2d.fromRotations(0)));
    m_leftFlightStick
        .B5()
        .onTrue(orchestraSubsystem.runOnce(orchestraSubsystem::togglePlayFromStart));
    m_leftFlightStick
        .B6()
        .onTrue(orchestraSubsystem.runOnce(orchestraSubsystem::nextTrackAndPlay));
    m_leftFlightStick
        .B4()
        .onTrue(orchestraSubsystem.runOnce(orchestraSubsystem::previousTrackAndPlay));
  }

  // private void setSwerveCommands() {
  // SwerveSubsystem swerveSubsystem = SwerveSubsystem.GetInstance();

  // swerveSubsystem.setDefaultCommand(new SwerveMoveTeleop(swerveSubsystem,
  // m_flightModule));

  // m_rightFlightStick
  // .B5()
  // .onTrue(swerveSubsystem.runOnce(() -> {
  // swerveSubsystem.resetGyro(0);
  // }));
  // }

  // private void setTurretCommands() {
  // TurretSubsystem.GetInstance().setDefaultCommand(
  // new ContinuousAimCommand(
  // () -> AimPoint.getTarget(GlobalPosition.Get())));
  // }

  public Command getAutonomousCommand() {
    return new Command() {
    };
  }

  public void onAnyModeStart() {
    // TurretSubsystem.GetInstance().reset();
    // var position = GlobalPosition.Get();
    // if (position != null) {
    // AHRSGyro.GetInstance().setAngleAdjustment(position.getRotation().getDegrees());
    // OdometrySubsystem.GetInstance().setOdometryPosition(position);
    // }

    // if (BotConstants.currentMode == BotConstants.Mode.REAL) {
    // PublicationSubsystem.addDataClasses(
    // OdometrySubsystem.GetInstance(),
    // AHRSGyro.GetInstance());
    // }
  }
}
