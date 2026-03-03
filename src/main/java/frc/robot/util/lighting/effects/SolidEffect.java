package frc.robot.util.lighting.effects;

import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightEffect;

public class SolidEffect extends LightEffect<Void> {
  private final LedColor color;

  public SolidEffect(LedRange range, LedColor color, int priority, BlendMode blendMode) {
    super(range, priority, blendMode);
    this.color = color;
  }

  @Override
  public LedColor sample(int ledIndex, double nowSeconds) {
    return color;
  }
}
