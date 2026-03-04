package frc.robot.command.lighting;

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
  private static final LedRange kAutonomousRangeL = new LedRange(15, 75);
  private static final LedRange kAutonomousRangeR = new LedRange(90, 150);
  private static final LedColor kSolidColor = new LedColor(0, 255, 0, 0);
  private static final double kHz = 70;
  private static final int kWidth = 30;

  private final LightsApi lightsApi;
  private EffectHandle<Void> chaseHandleL;
  private EffectHandle<Void> solidHandleL;

  private EffectHandle<Void> chaseHandleR;
  private EffectHandle<Void> solidHandleR;

  public AutonomousStateLighting() {
    this(LightsSubsystem.GetInstance());
  }

  public AutonomousStateLighting(LightsSubsystem lightsSubsystem) {
    super();
    this.lightsApi = lightsSubsystem;
  }

  @Override
  public void initialize() {
    chaseHandleL = lightsApi.addChase(
        kAutonomousRangeL,
        kChaseColor,
        kWidth,
        kHz,
        true,
        10,
        BlendMode.OVERWRITE);

    solidHandleL = lightsApi.addSolid(
        kAutonomousRangeL,
        kSolidColor,
        20,
        BlendMode.OVERWRITE);

    chaseHandleR = lightsApi.addChase(
        kAutonomousRangeR,
        kChaseColor,
        kWidth,
        kHz,
        true,
        10,
        BlendMode.OVERWRITE);

    solidHandleR = lightsApi.addSolid(
        kAutonomousRangeR,
        kSolidColor,
        20,
        BlendMode.OVERWRITE);
  }

  @Override
  public void execute() {
    if (PathPlannerSubsystem.GetInstance().currentAutoCommand == null) {
      lightsApi.setEnabled(chaseHandleL, false);
      lightsApi.setEnabled(chaseHandleR, false);

      lightsApi.setEnabled(solidHandleL, true);
      lightsApi.setEnabled(solidHandleR, true);

      return;
    }

    if (PathPlannerSubsystem.GetInstance().currentAutoCommand.isScheduled()) {
      lightsApi.setEnabled(chaseHandleL, true);
      lightsApi.setEnabled(solidHandleL, false);

      lightsApi.setEnabled(chaseHandleR, true);
      lightsApi.setEnabled(solidHandleR, false);
    } else {
      lightsApi.setEnabled(chaseHandleL, false);
      lightsApi.setEnabled(solidHandleL, true);

      lightsApi.setEnabled(chaseHandleR, false);
      lightsApi.setEnabled(solidHandleR, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (chaseHandleL != null) {
      lightsApi.removeEffect(chaseHandleL);
      lightsApi.removeEffect(solidHandleL);
      lightsApi.removeEffect(chaseHandleR);
      lightsApi.removeEffect(solidHandleR);
      chaseHandleL = null;
      solidHandleL = null;
      chaseHandleR = null;
      solidHandleR = null;
    }
  }
}
