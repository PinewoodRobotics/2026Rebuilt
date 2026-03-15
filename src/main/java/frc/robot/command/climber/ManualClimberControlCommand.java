package frc.robot.command.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ClimberSubsystem;

public class ManualClimberControlCommand extends Command {
  private final ClimberSubsystem m_climberSubsystem;
  private final DoubleSupplier m_heightPercentSupplier;
  private final DoubleSupplier m_wristVoltagePercentSupplier;

  public ManualClimberControlCommand(
      ClimberSubsystem climberSubsystem,
      DoubleSupplier heightPercentSupplier,
      DoubleSupplier wristVoltagePercentSupplier) {
    m_climberSubsystem = climberSubsystem;
    m_heightPercentSupplier = heightPercentSupplier;
    m_wristVoltagePercentSupplier = wristVoltagePercentSupplier;
    addRequirements(climberSubsystem);
  }

  @Override
  public void execute() {
    m_climberSubsystem.setHeightVoltagePercent(normalizePercentInput(m_heightPercentSupplier.getAsDouble()));
    m_climberSubsystem.setWristVoltagePercent(normalizePercentInput(m_wristVoltagePercentSupplier.getAsDouble()));
  }

  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stopHeightMotor();
    m_climberSubsystem.stopWristMotor();
  }

  private static double normalizePercentInput(double rawValue) {
    if (rawValue >= 0.0 && rawValue <= 1.0) {
      return rawValue;
    }

    return MathUtil.clamp((rawValue + 1.0) / 2.0, 0.0, 1.0);
  }
}
