package frc.robot.command.lighting;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.command.util.PollingCommand.IdCommand;
import frc.robot.subsystem.LightsSubsystem;
import frc.robot.subsystem.PathPlannerSubsystem;
import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.EffectHandle;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightsApi;

public class AutonomousStateLighting extends IdCommand {
  private static final LedColor kChaseColor = new LedColor(255, 0, 0, 0);
  private static final LedRange kAutonomousRange = new LedRange(0, 100);
  private static final LedColor kSolidColor = new LedColor(0, 255, 0, 0);

  private final LightsApi lightsApi;
  private EffectHandle<Void> chaseHandle;
  private EffectHandle<Void> solidHandle;

  public AutonomousStateLighting() {
    this(LightsSubsystem.GetInstance());
  }

  public AutonomousStateLighting(LightsSubsystem lightsSubsystem) {
    super();
    this.lightsApi = lightsSubsystem;
  }

  @Override
  public void initialize() {
    chaseHandle = lightsApi.addChase(
        kAutonomousRange,
        kChaseColor,
        10,
        10,
        true,
        10,
        BlendMode.OVERWRITE);

    solidHandle = lightsApi.addSolid(
        kAutonomousRange,
        kSolidColor,
        20,
        BlendMode.OVERWRITE);
  }

  @Override
  public void execute() {
    if (PathPlannerSubsystem.GetInstance().currentAutoCommand.isScheduled()) {
      lightsApi.setEnabled(chaseHandle, true);
      lightsApi.setEnabled(solidHandle, false);
    } else {
      lightsApi.setEnabled(chaseHandle, false);
      lightsApi.setEnabled(solidHandle, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (chaseHandle != null) {
      lightsApi.removeEffect(chaseHandle);
      chaseHandle = null;
    }
  }
}
