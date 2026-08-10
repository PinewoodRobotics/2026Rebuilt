package frc.robot.command.lighting;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.ShooterConstants;
import frc.robot.subsystem.LightsSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.EffectHandle;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightsApi;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import pwrup.frc.core.controller.FlightStick;

/**
 * Infinitely-running command that reads the speed slider and updates the
 * shooter speed display.
 */
public class ShooterSpeedLighting extends Command {
  private static final LedRange SHOOTER_BAR_RANGE = new LedRange(8, 67);
  private static final LedColor TARGET_FILL_COLOR = new LedColor(0, 255, 0, 0);
  private static final LedColor TARGET_EMPTY_COLOR = new LedColor(15, 15, 15, 0);
  private static final LedColor ACTUAL_FILL_COLOR = new LedColor(100, 0, 255, 0);

  private static AngularVelocity targetShooterSpeed = ShooterConstants.kShooterMinVelocity;

  private final LightsApi lightsApi;
  private final DoubleSupplier sliderSupplier;
  private final DoubleSupplier actualShooterRpsSupplier;

  private EffectHandle<Double> targetBarHandle;
  private EffectHandle<Double> actualBarHandle;

  public ShooterSpeedLighting(FlightStick flightStick) {
    this(LightsSubsystem.GetInstance(), flightStick);
  }

  public ShooterSpeedLighting(LightsSubsystem lightsSubsystem, FlightStick flightStick) {
    this(
        lightsSubsystem,
        flightStick::getRightSlider,
        () -> ShooterSubsystem.GetInstance().getCurrentShooterVelocity().in(Units.RotationsPerSecond));
  }

  ShooterSpeedLighting(
      LightsApi lightsApi,
      DoubleSupplier sliderSupplier,
      DoubleSupplier actualShooterRpsSupplier) {
    super();
    this.lightsApi = lightsApi;
    this.sliderSupplier = sliderSupplier;
    this.actualShooterRpsSupplier = actualShooterRpsSupplier;
  }

  @Override
  public void initialize() {
    targetBarHandle = lightsApi.addProgressBar(
        SHOOTER_BAR_RANGE,
        TARGET_FILL_COLOR,
        TARGET_EMPTY_COLOR,
        10,
        BlendMode.OVERWRITE);

    actualBarHandle = lightsApi.addProgressBar(
        SHOOTER_BAR_RANGE,
        ACTUAL_FILL_COLOR,
        null,
        20,
        BlendMode.ADD);
  }

  @Override
  public void execute() {
    double sliderPercent = Math.max(0, Math.min(1, (sliderSupplier.getAsDouble() + 1) / 2));
    double min = ShooterConstants.kShooterMinVelocity.in(Units.RotationsPerSecond);
    double max = ShooterConstants.kShooterMaxVelocity.in(Units.RotationsPerSecond);
    targetShooterSpeed = Units.RotationsPerSecond.of(min + (max - min) * sliderPercent);

    if (targetBarHandle != null) {
      lightsApi.setProgress(targetBarHandle, sliderPercent);
    }

    double actualPercent = actualShooterRpsSupplier.getAsDouble() / max;
    if (actualBarHandle != null) {
      lightsApi.setProgress(actualBarHandle, Math.max(0, Math.min(1, actualPercent)));
    }

    Logger.recordOutput("Shooter/NeededSpeed", targetShooterSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static AngularVelocity getTargetShooterSpeed() {
    return targetShooterSpeed;
  }
}
