package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command.ClimbAuto;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.command.climber.CalibrateClimberCommand;
import frc.robot.command.climber.ManualClimberControlCommand;
import frc.robot.command.intake.IntakeCommand;
import frc.robot.command.lighting.ShootingLighting;
import frc.robot.command.scoring.ContinuousAimCommand;
import frc.robot.command.scoring.ManualAimCommand;
import frc.robot.command.shooting.ContinuousManualShooter;
import frc.robot.command.shooting.ContinuousShooter;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import frc.robot.constant.BotConstants;
import frc.robot.constant.ClimberConstants;
import frc.robot.constant.IntakeConstants.WristRaiseLocation;
import frc.robot.constant.ShooterConstants;
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
import frc.robot.subsystem.VisionSubsystem;
import frc.robot.subsystem.ClimberSubsystem;
import frc.robot.util.AimPoint;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;
import pwrup.frc.core.controller.OperatorPanel;

public class RobotContainer {
  private static BooleanSupplier shooterArmedSupplierForHud = () -> false;

  final OperatorPanel m_operatorPanel = new OperatorPanel(1);
  final FlightStick m_leftFlightStick = new FlightStick(2);
  final FlightStick m_rightFlightStick = new FlightStick(3);
  final FlightModule m_flightModule = new FlightModule(
      m_leftFlightStick,
      m_rightFlightStick);

  public RobotContainer() {
    shooterArmedSupplierForHud = () -> m_operatorPanel.metalSwitchDown().getAsBoolean();
    GlobalPosition.GetInstance();

    UnifiedGyro.GetInstance();
    OdometrySubsystem.GetInstance(UnifiedGyro.GetInstance());
    SwerveSubsystem.GetInstance(UnifiedGyro.GetInstance());
    VisionSubsystem.GetInstance();

    TurretSubsystem.GetInstance();
    ShooterSubsystem.GetInstance();
    IndexSubsystem.GetInstance();
    IntakeSubsystem.GetInstance();

    MatchStatusSubsystem.GetInstance();

    var lights = LightsSubsystem.GetInstance();
    lights.addLightsCommand(new ShootingLighting(() -> !m_operatorPanel.metalSwitchDown().getAsBoolean()));

    // Preload PathPlanner before mode transitions to avoid first-enable auto hitch.
    PathPlannerSubsystem.GetInstance();

    setSwerveCommands();
    setTurretCommands();
    setShooterCommands();
    setIntakeCommands();
    setClimberCommands();

    BotConstants.SetAlliance();
  }

  private void setSwerveCommands() {
    SwerveSubsystem swerveSubsystem = SwerveSubsystem.GetInstance();
    BooleanSupplier isShootingSupplier = () -> m_operatorPanel.metalSwitchDown().getAsBoolean();

    swerveSubsystem
        .setDefaultCommand(
            new SwerveMoveTeleop(swerveSubsystem, m_flightModule, Optional.of(isShootingSupplier)));

    // Toggle gps-based driving assist features
    m_leftFlightStick.B5().onTrue(new InstantCommand(() -> {
      swerveSubsystem.setGpsAssist(!swerveSubsystem.isGpsAssist());
      Logger.recordOutput("SwerveSubsystem/GPSAssistFeaturesEnabled", swerveSubsystem.isGpsAssist());
    }));

    // Reset gyro rotation of the swerve dynamically
    m_rightFlightStick
        .B5()
        .onTrue(swerveSubsystem.runOnce(() -> {
          swerveSubsystem.resetDriverRelative();
        }));

    // Reset gyro rotation of the swerve to the global position
    m_rightFlightStick
        .B6()
        .onTrue(swerveSubsystem.runOnce(() -> {
          swerveSubsystem.resetDriverRelative(new Rotation2d());
        }));

    // Reset gyro rotation everywhere
    m_operatorPanel.blackButton().onTrue(new InstantCommand(() -> {
      var odometry = OdometrySubsystem.GetInstance();
      boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
      Rotation2d awayFromDriverStation = isRed ? Rotation2d.k180deg : Rotation2d.kZero;
      odometry.resetPose(new Pose2d(odometry.getPose().getTranslation(), awayFromDriverStation));
    }));
  }

  private void setTurretCommands() {
    var continuousAimCommand = new ContinuousAimCommand(
        () -> AimPoint.getTarget());

    var manualAimCommand = new ManualAimCommand(
        () -> ManualAimCommand.ReverseDirection(m_operatorPanel.getWheel()));

    TurretSubsystem.GetInstance().setDefaultCommand(Commands.either(
        continuousAimCommand,
        manualAimCommand,
        TurretSubsystem::getIsGpsAssistEnabled));

    m_operatorPanel.greenButton().onTrue(new InstantCommand(() -> {
      boolean gpsAssistEnabled = !ShooterSubsystem.getIsGpsAssistEnabled();
      ShooterSubsystem.setGpsAssistEnabled(gpsAssistEnabled);

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
    ClimberSubsystem climberSubsystem = ClimberSubsystem.GetInstance();
    climberSubsystem.setDefaultCommand(new ManualClimberControlCommand(
        climberSubsystem,
        m_leftFlightStick::getLeftSlider, false, true));

    m_operatorPanel.redButton().whileTrue(Commands.startEnd(
        () -> climberSubsystem.setVelocity(ClimberConstants.kManualDownVelocity),
        climberSubsystem::stopHeightMotor,
        climberSubsystem));

    m_leftFlightStick.B7().onTrue(new CalibrateClimberCommand(climberSubsystem));

    NamedCommands.registerCommand("MoveClimberUp",
        new ManualClimberControlCommand(climberSubsystem, () -> 1.0, true, false));

    NamedCommands.registerCommand("MoveClimberDown",
        new ManualClimberControlCommand(climberSubsystem, () -> 0.6, false, false));

    m_leftFlightStick.B17().whileTrue(new ClimbAuto(ClimbAuto.ClosePath.LEFT));
    m_leftFlightStick.B16().whileTrue(new ClimbAuto(ClimbAuto.ClosePath.RIGHT));
  }

  private void setIntakeCommands() {
    IntakeSubsystem intakeSubsystem = IntakeSubsystem.GetInstance();
    IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem,
        () -> m_operatorPanel.metalSwitchDown().getAsBoolean() || m_rightFlightStick.trigger().getAsBoolean(),
        () -> m_rightFlightStick.B17().getAsBoolean());
    Trigger teleopEnabled = new Trigger(DriverStation::isTeleopEnabled);

    intakeSubsystem
        .setDefaultCommand(intakeCommand);

    m_operatorPanel.toggleWheelMiddle().and(teleopEnabled).onTrue(new InstantCommand(() -> {
      intakeCommand.setAlternateRaiseLocation(WristRaiseLocation.TOP);
    }));
    m_operatorPanel.toggleWheelMidDown().and(teleopEnabled).onTrue(new InstantCommand(() -> {
      intakeCommand.setAlternateRaiseLocation(WristRaiseLocation.MIDDLE);
    }));

    NamedCommands.registerCommand("IntakeBottomCommand",
        new IntakeCommand(intakeSubsystem, () -> true,
            () -> false, WristRaiseLocation.BOTTOM, true));

    NamedCommands.registerCommand("IntakeMiddleCommand",
        new IntakeCommand(intakeSubsystem, () -> false,
            () -> false, WristRaiseLocation.MIDDLE, true));

    NamedCommands.registerCommand("IntakeTopCommand",
        new IntakeCommand(intakeSubsystem, () -> false,
            () -> false, WristRaiseLocation.TOP, true));
  }

  private void setShooterCommands() {
    BooleanSupplier indexExtakeOverrideSupplier = () -> m_rightFlightStick.B17().getAsBoolean();
    var continuousShooter = new ContinuousShooter(() -> AimPoint.getTarget(), indexExtakeOverrideSupplier);
    var continuousManualShooter = new ContinuousManualShooter(
        ContinuousManualShooter.GetHeldSpeedSupplier(
            () -> m_leftFlightStick.A().getAsBoolean(),
            () -> m_leftFlightStick.B().getAsBoolean(),
            ShooterConstants.kShooterBaseSpeed,
            40.0),
        indexExtakeOverrideSupplier);
    Trigger shooterEnabled = m_operatorPanel.metalSwitchDown().and(DriverStation::isTeleopEnabled);

    // Enable shooter with metal switch down. While up, run motor base speed.
    // When enabled, run indexer only when shooter up to speed.
    shooterEnabled
        .whileTrue(Commands.either(
            continuousShooter,
            continuousManualShooter,
            ShooterSubsystem::getIsGpsAssistEnabled))
        .negate()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(new InstantCommand(() -> {
          ShooterSubsystem.GetInstance().runMotorBaseSpeed();
        }));

    NamedCommands.registerCommand("ContinuousShooterCommand", new ContinuousShooter(() -> AimPoint.getTarget()));
  }

  public Command getAutonomousCommand() {
    return PathPlannerSubsystem.GetInstance().getAndInitAutoCommand(false);
  }

  public static boolean isShooterArmedForHud() {
    return shooterArmedSupplierForHud.getAsBoolean();
  }

  public void onAnyModeStart() {
    BotConstants.SetAlliance();
  }
}
