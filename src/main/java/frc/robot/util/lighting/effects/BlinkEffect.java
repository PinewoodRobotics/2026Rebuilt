package frc.robot.util.lighting.effects;

import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightEffect;

public class BlinkEffect extends LightEffect<Void> {
  private final LedColor onColor;
  private final LedColor offColor;
  private final double hz;

  public BlinkEffect(
      LedRange range,
      LedColor onColor,
      LedColor offColor,
      double hz,
      int priority,
      BlendMode blendMode) {
    super(range, priority, blendMode);
    this.onColor = onColor;
    this.offColor = offColor;
    this.hz = hz;
  }

  @Override
  public LedColor sample(int ledIndex, double nowSeconds) {
    if (hz <= 0) {
      return onColor;
    }
    double cycle = nowSeconds * hz;
    boolean on = (cycle - Math.floor(cycle)) < 0.5;
    return on ? onColor : offColor;
  }
}
