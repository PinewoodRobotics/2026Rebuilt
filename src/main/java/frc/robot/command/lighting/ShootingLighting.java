package frc.robot.command.lighting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.LightsSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.EffectHandle;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LightZone;
import frc.robot.util.lighting.LightsApi;
import java.util.function.BooleanSupplier;

public class ShootingLighting extends Command {
  private static final LedColor kWarmupPulseColor = new LedColor(255, 100, 0, 0);
  private static final LedColor kLowSpeedColor = new LedColor(255, 0, 0, 0);
  private static final LedColor kMidSpeedColor = new LedColor(255, 120, 0, 0);
  private static final LedColor kNearSpeedColor = new LedColor(255, 255, 0, 0);
  private static final LedColor kReadyColor = new LedColor(0, 255, 0, 0);
  private static final double kWarmupPulseHz = 7.0;
  private static final double kWarmupPulseMinScalar = 0.15;
  private static final double kWarmupPulseMaxScalar = 1.0;
  private static final double kLowSpeedChaseHz = 8.0;
  private static final double kMidSpeedChaseHz = 5.0;
  private static final double kNearSpeedBlinkHz = 5.5;
  private static final int kChaseWidth = 24;
  private static final int kPriority = 25;
  private static final double kMinTargetRps = 1e-6;
  private static final double kLowBandThreshold = 0.50;
  private static final double kNearBandThreshold = 0.90;

  private final LightsApi lightsApi;
  private final ShooterSubsystem shooterSubsystem;
  private final BooleanSupplier isMetalSwitchUpSupplier;

  private EffectHandle<Void> warmupPulseHandle;
  private EffectHandle<Void> lowSpeedChaseHandle;
  private EffectHandle<Void> midSpeedChaseHandle;
  private EffectHandle<Void> nearSpeedBlinkHandle;
  private EffectHandle<Void> readySolidHandle;

  public ShootingLighting(BooleanSupplier isMetalSwitchUpSupplier) {
    this(LightsSubsystem.GetInstance(), ShooterSubsystem.GetInstance(), isMetalSwitchUpSupplier);
  }

  ShootingLighting(
      LightsApi lightsApi,
      ShooterSubsystem shooterSubsystem,
      BooleanSupplier isMetalSwitchUpSupplier) {
    super();
    this.lightsApi = lightsApi;
    this.shooterSubsystem = shooterSubsystem;
    this.isMetalSwitchUpSupplier = isMetalSwitchUpSupplier;
  }

  @Override
  public void initialize() {
    var fullRange = LightZone.FULL_STRIP.range();
    warmupPulseHandle = lightsApi.addBreathe(
        fullRange,
        kWarmupPulseColor,
        kWarmupPulseHz,
        kWarmupPulseMinScalar,
        kWarmupPulseMaxScalar,
        kPriority,
        BlendMode.OVERWRITE);

    lowSpeedChaseHandle = lightsApi.addChase(
        fullRange,
        kLowSpeedColor,
        kChaseWidth,
        kLowSpeedChaseHz,
        true,
        kPriority,
        BlendMode.OVERWRITE);

    midSpeedChaseHandle = lightsApi.addChase(
        fullRange,
        kMidSpeedColor,
        kChaseWidth,
        kMidSpeedChaseHz,
        true,
        kPriority,
        BlendMode.OVERWRITE);

    nearSpeedBlinkHandle = lightsApi.addBlink(
        fullRange,
        kNearSpeedColor,
        new LedColor(0, 0, 0, 0),
        kNearSpeedBlinkHz,
        kPriority,
        BlendMode.OVERWRITE);

    readySolidHandle = lightsApi.addSolid(
        fullRange,
        kReadyColor,
        kPriority,
        BlendMode.OVERWRITE);
  }

  @Override
  public void execute() {
    boolean metalSwitchUp = isMetalSwitchUpSupplier.getAsBoolean();
    lightsApi.setEnabled(warmupPulseHandle, metalSwitchUp);
    lightsApi.setEnabled(lowSpeedChaseHandle, false);
    lightsApi.setEnabled(midSpeedChaseHandle, false);
    lightsApi.setEnabled(nearSpeedBlinkHandle, false);
    lightsApi.setEnabled(readySolidHandle, false);

    if (metalSwitchUp) {
      return;
    }

    double targetRps = shooterSubsystem.getRequestedShooterVelocityRps();
    double actualRps = shooterSubsystem.getCurrentShooterVelocityRps();
    double progress = targetRps <= kMinTargetRps ? 0.0 : actualRps / targetRps;
    if (progress < kLowBandThreshold) {
      lightsApi.setEnabled(lowSpeedChaseHandle, true);
      return;
    }
    if (progress < kNearBandThreshold) {
      lightsApi.setEnabled(midSpeedChaseHandle, true);
      return;
    }
    if (progress < 1.0) {
      lightsApi.setEnabled(nearSpeedBlinkHandle, true);
      return;
    }
    lightsApi.setEnabled(readySolidHandle, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    removeHandle(warmupPulseHandle);
    warmupPulseHandle = null;
    removeHandle(lowSpeedChaseHandle);
    lowSpeedChaseHandle = null;
    removeHandle(midSpeedChaseHandle);
    midSpeedChaseHandle = null;
    removeHandle(nearSpeedBlinkHandle);
    nearSpeedBlinkHandle = null;
    removeHandle(readySolidHandle);
    readySolidHandle = null;
  }

  private void removeHandle(EffectHandle<?> handle) {
    if (handle != null) {
      lightsApi.removeEffect(handle);
    }
  }
}
