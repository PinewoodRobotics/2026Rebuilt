package frc.robot.command.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ClimberSubsystem;

public class ManualClimberControlCommand extends Command {
  private final ClimberSubsystem m_climberSubsystem;
  private final DoubleSupplier m_heightUpPercentSupplier;
  private final DoubleSupplier m_heightDownPercentSupplier;
  private final DoubleSupplier m_wristVoltagePercentSupplier;

  public ManualClimberControlCommand(
      ClimberSubsystem climberSubsystem,
      DoubleSupplier heightUpPercentSupplier,
      DoubleSupplier heightDownPercentSupplier,
      DoubleSupplier wristVoltagePercentSupplier) {
    m_climberSubsystem = climberSubsystem;
    m_heightUpPercentSupplier = heightUpPercentSupplier;
    m_heightDownPercentSupplier = heightDownPercentSupplier;
    m_wristVoltagePercentSupplier = wristVoltagePercentSupplier;
    addRequirements(climberSubsystem);
  }

  @Override
  public void execute() {
    double upwardPercent = normalizePercentInput(m_heightUpPercentSupplier.getAsDouble());
    double downwardPercent = normalizePercentInput(m_heightDownPercentSupplier.getAsDouble());
    m_climberSubsystem.setHeightVoltagePercent(MathUtil.clamp(upwardPercent - downwardPercent, -1.0, 1.0));
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
