package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.command.SwerveMoveTeleop.AxisConstraint;
import frc.robot.command.lighting.AutonomousStateLighting;
import frc.robot.command.lighting.MorseCodeLighting;
import frc.robot.command.lighting.PulsingLightingCommand;
import frc.robot.command.lighting.ShooterSpeedLighting;
import frc.robot.command.lighting.TurretStateLighting;
import frc.robot.command.scoring.ContinuousAimCommand;
import frc.robot.command.scoring.ManualAimCommand;
import frc.robot.command.shooting.ShooterCommand;
import frc.robot.command.testing.IndexCommand;
import frc.robot.command.testing.IntakeCommand;
import frc.robot.command.testing.SetWristPos;
import frc.robot.constant.BotConstants;
import frc.robot.constant.IndexConstants;
import frc.robot.constant.IntakeConstants;
import frc.robot.constant.PathPlannerConstants;
import frc.robot.hardware.AHRSGyro;
import frc.robot.hardware.PigeonGyro;
import frc.robot.subsystem.CameraSubsystem;
import frc.robot.subsystem.GlobalPosition;
import frc.robot.subsystem.IndexSubsystem;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.LightsSubsystem;
import frc.robot.subsystem.OdometrySubsystem;
import frc.robot.subsystem.PathPlannerSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.SwerveSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.AimPoint;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;
import pwrup.frc.core.controller.OperatorPanel;
import pwrup.frc.core.online.PublicationSubsystem;

public class RobotContainer {
  final OperatorPanel m_operatorPanel = new OperatorPanel(1);
  final FlightStick m_leftFlightStick = new FlightStick(2);
  final FlightStick m_rightFlightStick = new FlightStick(3);
  final FlightModule m_flightModule = new FlightModule(
      m_leftFlightStick,
      m_rightFlightStick);

  public RobotContainer() {
    GlobalPosition.GetInstance();
    OdometrySubsystem.GetInstance();
    // AHRSGyro.GetInstance();
    PigeonGyro.GetInstance();
    SwerveSubsystem.GetInstance();
    CameraSubsystem.GetInstance();

    TurretSubsystem.GetInstance();
    // ShooterSubsystem.GetInstance();
    // IndexSubsystem.GetInstance();
    // LightsSubsystem.GetInstance();

    // IntakeSubsystem.GetInstance();

    // Initialize publication subsystem for sending data to Pi
    PublicationSubsystem.GetInstance(Robot.getCommunicationClient());
    PathPlannerSubsystem.GetInstance();

    // setIntakeCommands();

    setSwerveCommands();
    setTurretCommands();
    // setIndexCommands();
    // setShooterCommands();

    // setTestCommands();
    LightsSubsystem.GetInstance().addLightsCommand(
        new TurretStateLighting(),
        new AutonomousStateLighting(),
        new PulsingLightingCommand());
  }

  private void setTestCommands() {
    IndexSubsystem indexSubsystem = IndexSubsystem.GetInstance();
    IntakeSubsystem intakeSubsystem = IntakeSubsystem.GetInstance();
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
  }

  private void setSwerveCommands() {
    SwerveSubsystem swerveSubsystem = SwerveSubsystem.GetInstance();

    swerveSubsystem
        .setDefaultCommand(
            new SwerveMoveTeleop(swerveSubsystem, m_flightModule, PathPlannerConstants.kLanes,
                swerveSubsystem::getShouldAdjustVelocity));

    m_leftFlightStick.B5().onTrue(new InstantCommand(() -> {
      swerveSubsystem.setShouldAdjustVelocity(!swerveSubsystem.getShouldAdjustVelocity());
    }));

    m_rightFlightStick
        .B5()
        .onTrue(swerveSubsystem.runOnce(() -> {
          swerveSubsystem.resetGyro(0);
        }));

    new JoystickButton(
        m_operatorPanel,
        OperatorPanel.ButtonEnum.METALSWITCHDOWN.value).whileFalse(Commands.run(() -> {
          var position = GlobalPosition.Get();
          if (position != null) {
            PigeonGyro.GetInstance().resetRotation(position.getRotation());
          }
        })).onTrue(new InstantCommand(() -> {
          Logger.recordOutput("PigeonGyro/ResettingRotation", false);
        })).onFalse(new InstantCommand(() -> {
          Logger.recordOutput("PigeonGyro/ResettingRotation", true);
        }));

    Logger.recordOutput("PigeonGyro/ResettingRotation",
        !m_operatorPanel.getRawButton(OperatorPanel.ButtonEnum.METALSWITCHDOWN.value));
  }

  private void setTurretCommands() {

    TurretSubsystem.GetInstance().setDefaultCommand(
        new ContinuousAimCommand(
            () -> AimPoint.getTarget(GlobalPosition.Get())));

    /*
     * TurretSubsystem.GetInstance()
     * .setDefaultCommand(new ManualAimCommand(TurretSubsystem.GetInstance(), () ->
     * m_leftFlightStick.getTwist()));
     */
  }

  private void setIndexCommands() {
    IndexSubsystem indexSubsystem = IndexSubsystem.GetInstance();
    m_rightFlightStick.trigger().whileTrue(new IndexCommand(indexSubsystem, IndexConstants.kIndexMotorSpeed));
  }

  private void setIntakeCommands() {
    IntakeSubsystem intakeSubsystem = IntakeSubsystem.GetInstance();

    m_rightFlightStick.B5()
        .onTrue(new InstantCommand(() -> intakeSubsystem._toggleWristPosition()));
  }

  private void setShooterCommands() {
    ShooterSubsystem shooterSubsystem = ShooterSubsystem.GetInstance();

    m_rightFlightStick.trigger()
        .whileTrue(new ShooterCommand(shooterSubsystem, ShooterSpeedLighting::getTargetShooterSpeed));
  }

  public Command getAutonomousCommand() {
    return PathPlannerSubsystem.GetInstance().getAndInitAutoCommand(true);
  }

  public void onAnyModeStart() {
    var globalPosition = GlobalPosition.Get();
    if (globalPosition != null) {
      PigeonGyro.GetInstance().resetRotation(globalPosition.getRotation());
      OdometrySubsystem.GetInstance().setOdometryPosition(globalPosition);
    }

    PublicationSubsystem.addDataClasses(
        PigeonGyro.GetInstance(), OdometrySubsystem.GetInstance());

    /*
     * TurretSubsystem.GetInstance().reset();
     * var position = GlobalPosition.Get();
     * if (position != null) {
     * PigeonGyro.GetInstance().setYawDegrees(position.getRotation().getDegrees());
     * OdometrySubsystem.GetInstance().setOdometryPosition(position);
     * }
     * 
     * if (BotConstants.currentMode == BotConstants.Mode.REAL) {
     * PublicationSubsystem.addDataClasses(
     * OdometrySubsystem.GetInstance(),
     * PigeonGyro.GetInstance());
     * }
     */
  }
}
