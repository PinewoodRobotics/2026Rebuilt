package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.command.intake.IntakeCommand;
import frc.robot.command.scoring.ContinuousAimCommand;
import frc.robot.command.scoring.ManualAimCommand;
import frc.robot.command.shooting.ContinuousManualShooter;
import frc.robot.command.shooting.ContinuousShooter;
import frc.robot.command.testing.IndexCommand;
import frc.robot.constant.IndexConstants;
import frc.robot.constant.PathPlannerConstants;
import frc.robot.hardware.UnifiedGyro;
import frc.robot.subsystem.GlobalPosition;
import frc.robot.subsystem.IndexSubsystem;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.LightsSubsystem;
import frc.robot.subsystem.MatchStatusSubsystem;
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

  /**
   * When true, turret uses manual aim and shooter uses manual speed (slider).
   * Toggled by green button.
   */
  private boolean isManualScoringMode = false;

  public RobotContainer() {
    PublicationSubsystem.GetInstance(Robot.getCommunicationClient());

    GlobalPosition.GetInstance();

    UnifiedGyro.GetInstance();
    OdometrySubsystem.GetInstance(UnifiedGyro.GetInstance());
    SwerveSubsystem.GetInstance(UnifiedGyro.GetInstance());

    TurretSubsystem.GetInstance();
    ShooterSubsystem.GetInstance();
    IndexSubsystem.GetInstance();
    IntakeSubsystem.GetInstance();

    // Initialize publication subsystem for sending data to Pi
    PublicationSubsystem.GetInstance(Robot.getCommunicationClient());
    MatchStatusSubsystem.GetInstance();

    // setIntakeCommands();
    // PathPlannerSubsystem.GetInstance();

    setSwerveCommands();
    setTurretCommands();
    setShooterCommands();
    setIntakeCommands();
  }

  private void setSwerveCommands() {
    SwerveSubsystem swerveSubsystem = SwerveSubsystem.GetInstance();

    swerveSubsystem
        .setDefaultCommand(
            new SwerveMoveTeleop(swerveSubsystem, m_flightModule, PathPlannerConstants.kLanes,
                swerveSubsystem::getIsGpsAssist));

    // Toggle gps-based driving assist features
    m_leftFlightStick.B5().onTrue(new InstantCommand(() -> {
      swerveSubsystem.setGpsAssist(!swerveSubsystem.getIsGpsAssist());
      Logger.recordOutput("SwerveSubsystem/GPSAssistFeaturesEnabled", swerveSubsystem.getIsGpsAssist());
    }));

    // Reset gyro rotation of the swerve dynamically
    m_rightFlightStick
        .B5()
        .onTrue(swerveSubsystem.runOnce(() -> {
          swerveSubsystem.resetGyro(0);
        }));

    // Reset gyro rotation everywhere (including backend with button)
    m_operatorPanel.blackButton().whileFalse(Commands.run(() -> {
      var position = GlobalPosition.Get();
      if (position != null) {
        UnifiedGyro.GetInstance().resetRotation(position.getRotation());
      }
    })).onTrue(new InstantCommand(() -> {
      Logger.recordOutput("UnifiedGyro/ResettingRotation", false);
    })).onFalse(new InstantCommand(() -> {
      Logger.recordOutput("UnifiedGyro/ResettingRotation", true);
    }));
    Logger.recordOutput("UnifiedGyro/ResettingRotation", false);
  }

  private void setTurretCommands() {
    var continuousAimCommand = new ContinuousAimCommand(
        () -> AimPoint.getTarget());

    var manualAimCommand = new ManualAimCommand(
        () -> ContinuousManualShooter.ReverseDirection(m_operatorPanel.getWheel()));

    TurretSubsystem.GetInstance().setDefaultCommand(Commands.either(
        continuousAimCommand,
        manualAimCommand,
        TurretSubsystem::getIsGpsAssistEnabled));

    m_operatorPanel.greenButton().onTrue(new InstantCommand(() -> {
      TurretSubsystem.setGpsAssistEnabled(!TurretSubsystem.getIsGpsAssistEnabled());
      ShooterSubsystem.setGpsAssistEnabled(!ShooterSubsystem.getIsGpsAssistEnabled());

      var current = TurretSubsystem.GetInstance().getCurrentCommand();
      if (current != null) {
        current.cancel();
      }

      var currentShooterCommand = ShooterSubsystem.GetInstance().getCurrentCommand();
      if (currentShooterCommand != null) {
        currentShooterCommand.cancel();
      }

      Logger.recordOutput("TurretSubsystem/GPSAssistFeaturesEnabled", TurretSubsystem.getIsGpsAssistEnabled());
      Logger.recordOutput("ShooterSubsystem/GPSAssistFeaturesEnabled", ShooterSubsystem.getIsGpsAssistEnabled());
    }));

    NamedCommands.registerCommand("ContinuousAimCommand", new ContinuousAimCommand(() -> AimPoint.getTarget()));
  }

  private void setClimberCommands() {
    // TODO: Implement climber commands
    // NamedCommands.registerCommand("ClimberL1Command", new
    // ClimberCommand(climberSubsystem, () ->
    // m_rightFlightStick.trigger().getAsBoolean()));
  }

  private void setIntakeCommands() {
    IntakeSubsystem intakeSubsystem = IntakeSubsystem.GetInstance();

    intakeSubsystem
        .setDefaultCommand(new IntakeCommand(intakeSubsystem, () -> m_rightFlightStick.trigger().getAsBoolean(),
            () -> m_rightFlightStick.B17().getAsBoolean()));

    NamedCommands.registerCommand("IntakeCommand",
        new IntakeCommand(intakeSubsystem, () -> true,
            () -> false));
  }

  private void setShooterCommands() {
    var continuousShooter = new ContinuousShooter(() -> AimPoint.getTarget());
    var continuousManualShooter = new ContinuousManualShooter(
        ContinuousManualShooter.GetBaseSpeedSupplier(m_rightFlightStick::getRightSlider));

    // Enable shooter with metal switch down. While up, run motor base speed.
    // When enabled, run indexer only when shooter up to speed.
    m_operatorPanel.metalSwitchDown()
        .whileTrue(Commands.either(
            continuousShooter,
            continuousManualShooter,
            ShooterSubsystem::getIsGpsAssistEnabled))
        .whileFalse(new InstantCommand(() -> {
          ShooterSubsystem.GetInstance().runMotorBaseSpeed();
        }));

    NamedCommands.registerCommand("ContinuousShooterCommand", new ContinuousShooter(() -> AimPoint.getTarget()));
  }

  public Command getAutonomousCommand() {
    return PathPlannerSubsystem.GetInstance().getAndInitAutoCommand(true);
  }

  public void onAnyModeStart() {
    PublicationSubsystem.ClearAll();
    UnifiedGyro.Register();
    PublicationSubsystem.addDataClass(OdometrySubsystem.GetInstance());

    /*
     * var globalPosition = GlobalPosition.Get();
     * if (globalPosition != null) {
     * PigeonGyro.GetInstance().resetRotation(globalPosition.getRotation());
     * OdometrySubsystem.GetInstance().setOdometryPosition(globalPosition);
     * }
     * 
     * PublicationSubsystem.addDataClasses(
     * PigeonGyro.GetInstance(), OdometrySubsystem.GetInstance());
     */

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
