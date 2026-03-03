package frc.robot.util.lighting.effects;

import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightEffect;

public class ProgressBarEffect extends LightEffect<Double> {
  private final LedColor fillColor;
  private final LedColor emptyColorOrNull;
  private double progress01;

  public ProgressBarEffect(
      LedRange range,
      LedColor fillColor,
      LedColor emptyColorOrNull,
      int priority,
      BlendMode blendMode) {
    super(range, priority, blendMode);
    this.fillColor = fillColor;
    this.emptyColorOrNull = emptyColorOrNull;
    this.progress01 = 0.0;
  }

  @Override
  public boolean setProgress(double progress01) {
    this.progress01 = LedColor.clamp01(progress01);
    return true;
  }

  @Override
  public boolean updateInput(Double inputArg) {
    if (inputArg == null) {
      return false;
    }
    return setProgress(inputArg);
  }

  @Override
  public LedColor sample(int ledIndex, double nowSeconds) {
    int length = getRange().length();
    int localIndex = ledIndex - getRange().startInclusive();
    int filledCount = (int) Math.round(progress01 * length);
    filledCount = Math.max(0, Math.min(length, filledCount));
    boolean filled = localIndex < filledCount;
    if (filled) {
      return fillColor;
    }
    return emptyColorOrNull;
  }
}
