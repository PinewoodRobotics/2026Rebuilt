package frc.robot.command.climber;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.ClimberConstants;
import frc.robot.subsystem.ClimberSubsystem;

public class ManualClimberControlCommand extends Command {
  private final ClimberSubsystem m_climberSubsystem;
  private final DoubleSupplier m_heightSliderSupplier;

  public ManualClimberControlCommand(
      ClimberSubsystem climberSubsystem,
      DoubleSupplier heightSliderSupplier) {
    m_climberSubsystem = climberSubsystem;
    m_heightSliderSupplier = heightSliderSupplier;
    addRequirements(climberSubsystem);
  }

  @Override
  public void execute() {
    double sliderPosition = normalizeSliderInput(m_heightSliderSupplier.getAsDouble());
    double targetHeightMeters = MathUtil.interpolate(
        ClimberConstants.kMinHeight.in(Meters),
        ClimberConstants.kMaxHeight.in(Meters),
        sliderPosition);
    m_climberSubsystem.setHeight(Meters.of(targetHeightMeters));
  }

  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stopHeightMotor();
  }

  private static double normalizeSliderInput(double rawValue) {
    if (rawValue >= 0.0 && rawValue <= 1.0) {
      return rawValue;
    }

    return MathUtil.clamp((rawValue + 1.0) / 2.0, 0.0, 1.0);
  }
}
