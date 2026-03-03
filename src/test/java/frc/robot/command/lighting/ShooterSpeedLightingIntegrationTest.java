package frc.robot.command.lighting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.Units;
import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.EffectHandle;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightsApi;
import frc.robot.util.lighting.LightsEngine;
import frc.robot.util.lighting.effects.ProgressBarEffect;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class ShooterSpeedLightingIntegrationTest {
  @Test
  void initializeAndExecuteRegistersAndUpdatesBothBars() {
    FakeLightsApi fakeLights = new FakeLightsApi();
    ShooterSpeedLighting command = new ShooterSpeedLighting(
        fakeLights,
        () -> 0.0,
        () -> 20.0);

    command.initialize();
    command.execute();

    assertEquals(2, fakeLights.createdHandles.size());
    assertEquals(2, fakeLights.progressByHandle.size());
    assertTrue(containsApproxValue(fakeLights.progressByHandle, 0.5));
    assertTrue(containsApproxValue(fakeLights.progressByHandle, 0.2));
    assertEquals(50.0, ShooterSpeedLighting.getTargetShooterSpeed().in(Units.RotationsPerSecond), 1e-9);
  }

  private static boolean containsApproxValue(Map<EffectHandle<Double>, Double> values, double expected) {
    for (double value : values.values()) {
      if (Math.abs(value - expected) < 1e-9) {
        return true;
      }
    }
    return false;
  }

  private static class FakeLightsApi implements LightsApi {
    private final LightsEngine engine = new LightsEngine(400, 48);
    private final List<EffectHandle<Double>> createdHandles = new ArrayList<>();
    private final Map<EffectHandle<Double>, Double> progressByHandle = new HashMap<>();

    @Override
    public EffectHandle<Double> addProgressBar(
        LedRange range,
        LedColor fill,
        LedColor emptyOrNull,
        int priority,
        BlendMode blend) {
      EffectHandle<Double> handle =
          engine.addEffect(new ProgressBarEffect(range, fill, emptyOrNull, priority, blend));
      createdHandles.add(handle);
      return handle;
    }

    @Override
    public boolean setProgress(EffectHandle<?> handle, double progress01) {
      @SuppressWarnings("unchecked")
      EffectHandle<Double> typed = (EffectHandle<Double>) handle;
      progressByHandle.put(typed, progress01);
      return engine.setProgress(handle, progress01);
    }

    @Override
    public <T> boolean setInput(EffectHandle<T> handle, T inputArg) {
      return engine.setInput(handle, inputArg);
    }

    @Override
    public EffectHandle<Void> addSolid(LedRange range, LedColor color, int priority, BlendMode blend) {
      throw new UnsupportedOperationException();
    }

    @Override
    public EffectHandle<Void> addBlink(LedRange range, LedColor on, LedColor off, double hz, int priority,
        BlendMode blend) {
      throw new UnsupportedOperationException();
    }

    @Override
    public EffectHandle<Void> addBreathe(LedRange range, LedColor color, double hz, double minScalar,
        double maxScalar, int priority,
        BlendMode blend) {
      throw new UnsupportedOperationException();
    }

    @Override
    public EffectHandle<Void> addChase(LedRange range, LedColor color, int width, double hz, boolean wrap,
        int priority,
        BlendMode blend) {
      throw new UnsupportedOperationException();
    }

    @Override
    public EffectHandle<Void> addRainbow(LedRange range, double hz, double brightness, int priority, BlendMode blend) {
      throw new UnsupportedOperationException();
    }

    @Override
    public EffectHandle<Void> addLarsonScanner(LedRange range, LedColor color, int eyeWidth, int trailLength,
        double hz,
        int priority,
        BlendMode blend) {
      throw new UnsupportedOperationException();
    }

    @Override
    public EffectHandle<Double> addConvergingArrows(LedRange range, LedColor color, boolean wedge, int priority,
        BlendMode blend) {
      throw new UnsupportedOperationException();
    }

    @Override
    public boolean setEnabled(EffectHandle<?> handle, boolean enabled) {
      return engine.setEnabled(handle, enabled);
    }

    @Override
    public boolean setPriority(EffectHandle<?> handle, int priority) {
      return engine.setPriority(handle, priority);
    }

    @Override
    public boolean removeEffect(EffectHandle<?> handle) {
      return engine.removeEffect(handle);
    }

    @Override
    public void clearEffects() {
      engine.clearEffects();
    }
  }
}
