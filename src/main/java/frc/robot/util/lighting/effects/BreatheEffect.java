package frc.robot.util.lighting.effects;

import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightEffect;

public class BreatheEffect extends LightEffect<Void> {
  private final LedColor color;
  private final double hz;
  private final double minScalar;
  private final double maxScalar;

  public BreatheEffect(
      LedRange range,
      LedColor color,
      double hz,
      double minScalar,
      double maxScalar,
      int priority,
      BlendMode blendMode) {
    super(range, priority, blendMode);
    this.color = color;
    this.hz = hz;
    this.minScalar = LedColor.clamp01(Math.min(minScalar, maxScalar));
    this.maxScalar = LedColor.clamp01(Math.max(minScalar, maxScalar));
  }

  @Override
  public LedColor sample(int ledIndex, double nowSeconds) {
    if (hz <= 0) {
      return color.scale(maxScalar);
    }
    double wave01 = 0.5 + 0.5 * Math.sin(2.0 * Math.PI * hz * nowSeconds);
    double scalar = minScalar + (maxScalar - minScalar) * wave01;
    return color.scale(scalar);
  }
}
