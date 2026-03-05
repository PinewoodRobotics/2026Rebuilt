package frc.robot.command.lighting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.LEDConstants;
import frc.robot.subsystem.LightsSubsystem;
import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.EffectHandle;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightsApi;

public class PulsingLightingCommand extends Command {
  private static final int kPulseStartIndex = 150;
  private static final LedRange kPulseRange = new LedRange(kPulseStartIndex, LEDConstants.ledEndIndex);
  private static final LedColor kPulseColor = new LedColor(255, 100, 0, 0);
  private static final double kHz = 5;
  private static final double kMinScalar = 0.15;
  private static final double kMaxScalar = 1.0;
  private static final int kPriority = 15;

  private final LightsApi lightsApi;
  private EffectHandle<Void> breatheHandle;

  public PulsingLightingCommand() {
    this(LightsSubsystem.GetInstance());
  }

  public PulsingLightingCommand(LightsSubsystem lightsSubsystem) {
    super();
    this.lightsApi = lightsSubsystem;
  }

  @Override
  public void initialize() {
    breatheHandle = lightsApi.addBreathe(
        kPulseRange,
        kPulseColor,
        kHz,
        kMinScalar,
        kMaxScalar,
        kPriority,
        BlendMode.OVERWRITE);
  }

  @Override
  public void execute() {
    // Breathe runs on its own; no per-cycle updates needed.
  }

  @Override
  public void end(boolean interrupted) {
    if (breatheHandle != null) {
      lightsApi.removeEffect(breatheHandle);
      breatheHandle = null;
    }
  }
}
