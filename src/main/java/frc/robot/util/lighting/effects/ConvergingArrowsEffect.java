package frc.robot.util.lighting.effects;

import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightEffect;

/**
 * Aiming reticle: two arrows that converge toward the center as "exactness" goes from 0 to 1.
 * Left arrow "---->" grows from the left toward center; right arrow "<----" grows from the right.
 * Drive via {@link #setProgress(double)} with 0 = spread apart, 1 = meeting at center (on target).
 */
public class ConvergingArrowsEffect extends LightEffect<Double> {
  private final LedColor color;
  private final boolean wedge; // true = bright at tip, dim at base (arrow shape)
  private double exactness01;

  public ConvergingArrowsEffect(
      LedRange range,
      LedColor color,
      boolean wedge,
      int priority,
      BlendMode blendMode) {
    super(range, priority, blendMode);
    this.color = color;
    this.wedge = wedge;
    this.exactness01 = 0.0;
  }

  @Override
  public boolean setProgress(double progress01) {
    this.exactness01 = LedColor.clamp01(progress01);
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

    if (length <= 0) {
      return null;
    }

    double center = (length - 1) * 0.5;
    // Left arrow tip: at 0 when exactness=0, at center when exactness=1
    double tipLeft = exactness01 * center;
    // Right arrow tip: at length-1 when exactness=0, at center when exactness=1
    double tipRight = (length - 1) - exactness01 * ((length - 1) - center);

    // Left arrow: indices 0..tipLeft (inclusive in continuous sense)
    if (localIndex <= tipLeft + 0.5) {
      if (tipLeft <= 0) {
        return null;
      }
      if (wedge) {
        double t = tipLeft > 0 ? localIndex / tipLeft : 1.0;
        t = LedColor.clamp01(t);
        return color.scale(t);
      }
      return color;
    }

    // Right arrow: indices tipRight..length-1
    if (localIndex >= tipRight - 0.5) {
      double rightSpan = (length - 1) - tipRight;
      if (rightSpan <= 0) {
        return null;
      }
      if (wedge) {
        double t = rightSpan > 0 ? ((length - 1) - localIndex) / rightSpan : 1.0;
        t = LedColor.clamp01(t);
        return color.scale(t);
      }
      return color;
    }

    return null;
  }
}
