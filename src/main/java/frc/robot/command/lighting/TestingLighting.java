package frc.robot.command.lighting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.LEDConstants;
import frc.robot.subsystem.LightsSubsystem;
import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.EffectHandle;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightZone;
import frc.robot.util.lighting.LightsApi;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Maps physical LEDs to indices: the strip is a dim background; a bright marker
 * moves toward
 * lower-numbered LEDs at 5 LEDs per second while {@code advanceButton} is held.
 */
public class TestingLighting extends Command {
  /** Physical LEDs per second when the advance button is held. */
  private static final double kLedsPerSecond = 5.0;

  private static final LedRange kFullRange = LightZone.FULL_STRIP.range();
  private static final LedColor kBackgroundColor = new LedColor(25, 25, 35, 0);
  private static final LedColor kHighlightColor = new LedColor(0, 255, 180, 0);
  private static final int kBackgroundPriority = 10;
  private static final int kHighlightPriority = 60;

  private final LightsApi lightsApi;
  private final BooleanSupplier advanceButton;

  private EffectHandle<Void> backgroundHandle;
  private EffectHandle<Void> highlightHandle;
  private double headPosition;
  private double lastTimeSeconds;
  private int lastHighlightIndex = Integer.MIN_VALUE;

  public TestingLighting(BooleanSupplier advanceButton) {
    this(LightsSubsystem.GetInstance(), advanceButton);
  }

  public TestingLighting(LightsApi lightsApi, BooleanSupplier advanceButton) {
    super();
    this.lightsApi = lightsApi;
    this.advanceButton = advanceButton;
  }

  @Override
  public void initialize() {
    lastTimeSeconds = Timer.getFPGATimestamp();
    headPosition = LEDConstants.ledEndIndex;
    lastHighlightIndex = Integer.MIN_VALUE;
    backgroundHandle = lightsApi.addSolid(kFullRange, kBackgroundColor, kBackgroundPriority, BlendMode.OVERWRITE);
    syncHighlightEffect(true);
  }

  private void syncHighlightEffect(boolean force) {
    int idx = clampIndex((int) Math.round(headPosition));
    if (!force && idx == lastHighlightIndex) {
      return;
    }
    if (highlightHandle != null) {
      lightsApi.removeEffect(highlightHandle);
      highlightHandle = null;
    }
    highlightHandle = lightsApi.addSolid(LedRange.single(idx), kHighlightColor, kHighlightPriority,
        BlendMode.OVERWRITE);
    lastHighlightIndex = idx;
    Logger.recordOutput("TestingLighting/HighlightLedIndex", idx);
  }

  private static int clampIndex(int idx) {
    return Math.max(LEDConstants.ledStartIndex, Math.min(LEDConstants.ledEndIndex, idx));
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimeSeconds;
    lastTimeSeconds = now;

    if (advanceButton.getAsBoolean()) {
      headPosition -= kLedsPerSecond * dt;
      headPosition = Math.max(LEDConstants.ledStartIndex, headPosition);
    }

    syncHighlightEffect(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (highlightHandle != null) {
      lightsApi.removeEffect(highlightHandle);
      highlightHandle = null;
    }
    if (backgroundHandle != null) {
      lightsApi.removeEffect(backgroundHandle);
      backgroundHandle = null;
    }
    lastHighlightIndex = Integer.MIN_VALUE;
  }
}
