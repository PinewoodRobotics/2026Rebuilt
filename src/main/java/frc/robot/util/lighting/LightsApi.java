package frc.robot.util.lighting;

public interface LightsApi {
  EffectHandle<Double> addProgressBar(
      LedRange range,
      LedColor fill,
      LedColor emptyOrNull,
      int priority,
      BlendMode blend);

  EffectHandle<Void> addSolid(LedRange range, LedColor color, int priority, BlendMode blend);

  EffectHandle<Void> addBlink(LedRange range, LedColor on, LedColor off, double hz, int priority, BlendMode blend);

  EffectHandle<Void> addBreathe(
      LedRange range,
      LedColor color,
      double hz,
      double minScalar,
      double maxScalar,
      int priority,
      BlendMode blend);

  EffectHandle<Void> addChase(
      LedRange range,
      LedColor color,
      int width,
      double hz,
      boolean wrap,
      int priority,
      BlendMode blend);

  EffectHandle<Void> addRainbow(LedRange range, double hz, double brightness, int priority, BlendMode blend);

  EffectHandle<Void> addLarsonScanner(
      LedRange range,
      LedColor color,
      int eyeWidth,
      int trailLength,
      double hz,
      int priority,
      BlendMode blend);

  EffectHandle<Double> addConvergingArrows(
      LedRange range,
      LedColor color,
      boolean wedge,
      int priority,
      BlendMode blend);

  boolean setEnabled(EffectHandle<?> handle, boolean enabled);

  boolean setPriority(EffectHandle<?> handle, int priority);

  boolean removeEffect(EffectHandle<?> handle);

  void clearEffects();

  boolean setProgress(EffectHandle<?> handle, double progress01);

  <T> boolean setInput(EffectHandle<T> handle, T inputArg);
}
