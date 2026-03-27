package frc.robot.command.climber;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.ClimberConstants;
import frc.robot.subsystem.ClimberSubsystem;

public class ManualClimberControlCommand extends Command {
  private final ClimberSubsystem m_climberSubsystem;
  private final DoubleSupplier m_heightSliderSupplier;
  private boolean shouldEnd;
  private boolean isSlider;

  public ManualClimberControlCommand(
      ClimberSubsystem climberSubsystem,
      DoubleSupplier heightSliderSupplier, boolean shouldEnd, boolean isSlider) {
    this.m_climberSubsystem = climberSubsystem;
    this.m_heightSliderSupplier = heightSliderSupplier;
    this.shouldEnd = shouldEnd;
    this.isSlider = isSlider;
    addRequirements(climberSubsystem);
  }

  @Override
  public void execute() {
    double sliderPosition = isSlider ? normalizeSliderInput(m_heightSliderSupplier.getAsDouble())
        : m_heightSliderSupplier.getAsDouble();
    double targetHeightMeters = MathUtil.interpolate(
        ClimberConstants.kMinHeight.in(Meters),
        ClimberConstants.kMaxHeight.in(Meters),
        sliderPosition);
    m_climberSubsystem.setHeight(Meters.of(targetHeightMeters));

    Logger.recordOutput("Climber/ValueReq", m_heightSliderSupplier.getAsDouble());
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

  @Override
  public boolean isFinished() {
    return shouldEnd && m_climberSubsystem.atTarget();
  }
}
