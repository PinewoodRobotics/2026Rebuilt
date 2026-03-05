package frc.robot.command.lighting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.LightsSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.EffectHandle;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightsApi;

public class TurretStateLighting extends Command {
  private static final int kMaxAimTimeMs = 50;
  private static final LedColor kTargetColor = new LedColor(255, 0, 0, 0);
  private static final LedRange kTargetRange = new LedRange(75, 90);

  private final LightsApi lightsApi;
  private final TurretSubsystem turretSubsystem;

  private EffectHandle<Double> targetBarHandle;

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
    targetBarHandle = lightsApi.addConvergingArrows(
        kTargetRange,
        kTargetColor,
        true,
        10,
        BlendMode.OVERWRITE);
  }

  @Override
  public void execute() {
    int aimTimeLeftMs = turretSubsystem.getAimTimeLeftMs();
    double exactness = 1.0 - Math.min(1.0, (double) aimTimeLeftMs / kMaxAimTimeMs);
    lightsApi.setInput(targetBarHandle, exactness);
  }

  @Override
  public void end(boolean interrupted) {
    if (targetBarHandle != null) {
      lightsApi.removeEffect(targetBarHandle);
      targetBarHandle = null;
    }
  }
}
