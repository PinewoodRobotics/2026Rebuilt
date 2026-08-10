package frc.robot.command.lighting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.command.shooting.ContinuousShooter;
import frc.robot.subsystem.LightsSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.EffectHandle;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightsApi;

public class TurretStateLighting extends Command {
  private static final int kMaxAimTimeMs = 50;
  private static final int kInRangeThresholdMs = 20;
  private static final LedColor kTargetColorRed = new LedColor(255, 0, 0, 0);
  private static final LedColor kTargetColorGreen = new LedColor(0, 255, 0, 0);
  private static final LedRange kTargetRange = new LedRange(75, 90);

  private final LightsApi lightsApi;
  private final TurretSubsystem turretSubsystem;

  private EffectHandle<Double> redHandle;
  private EffectHandle<Double> greenHandle;

  public TurretStateLighting() {
    this(LightsSubsystem.GetInstance(), TurretSubsystem.GetInstance());
  }

  public TurretStateLighting(LightsSubsystem lightsSubsystem, TurretSubsystem turretSubsystem) {
    super();
    this.lightsApi = lightsSubsystem;
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public void initialize() {
    redHandle = lightsApi.addConvergingArrows(
        kTargetRange,
        kTargetColorRed,
        true,
        10,
        BlendMode.OVERWRITE);
    greenHandle = lightsApi.addConvergingArrows(
        kTargetRange,
        kTargetColorGreen,
        true,
        10,
        BlendMode.OVERWRITE);
  }

  @Override
  public void execute() {
    lightsApi.setEnabled(redHandle, !ContinuousShooter.isShooting());
    lightsApi.setEnabled(greenHandle, ContinuousShooter.isShooting());
  }

  @Override
  public void end(boolean interrupted) {
    if (redHandle != null) {
      lightsApi.removeEffect(redHandle);
      redHandle = null;
    }
    if (greenHandle != null) {
      lightsApi.removeEffect(greenHandle);
      greenHandle = null;
    }
  }
}
