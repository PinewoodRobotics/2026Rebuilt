package frc.robot.command.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.ClimberConstants;
import frc.robot.subsystem.ClimberSubsystem;

public class CalibrateClimberCommand extends Command {
  private final ClimberSubsystem m_climberSubsystem;
  private final Timer m_runtimeTimer = new Timer();
  private final Timer m_settledTimer = new Timer();
  private boolean m_hasDetectedCalibrationPoint = false;

  public CalibrateClimberCommand(ClimberSubsystem climberSubsystem) {
    m_climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    m_hasDetectedCalibrationPoint = false;
    m_runtimeTimer.restart();
    m_settledTimer.stop();
    m_settledTimer.reset();
    m_climberSubsystem.setVelocity(ClimberConstants.kCalibrationVelocity);
  }

  @Override
  public void execute() {
    m_climberSubsystem.setVelocity(ClimberConstants.kCalibrationVelocity);

    if (m_climberSubsystem.isCalibrationVelocitySettled()) {
      if (!m_settledTimer.isRunning()) {
        m_settledTimer.restart();
      }
    } else {
      m_settledTimer.stop();
      m_settledTimer.reset();
    }

    m_hasDetectedCalibrationPoint = m_runtimeTimer.hasElapsed(ClimberConstants.kCalibrationMinRuntimeSeconds)
        && m_settledTimer.hasElapsed(ClimberConstants.kCalibrationSettledTimeSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted && m_hasDetectedCalibrationPoint) {
      m_climberSubsystem.zeroHeightEncoder();
    }

    m_runtimeTimer.stop();
    m_settledTimer.stop();
    m_climberSubsystem.stopHeightMotor();
  }

  @Override
  public boolean isFinished() {
    return m_hasDetectedCalibrationPoint;
  }
}
