package frc.robot.util.lighting.effects;

import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightEffect;

public class ChaseEffect extends LightEffect<Void> {
  private final LedColor color;
  private final int width;
  private final double hz;
  private final boolean wrap;

  public ChaseEffect(
      LedRange range,
      LedColor color,
      int width,
      double hz,
      boolean wrap,
      int priority,
      BlendMode blendMode) {
    super(range, priority, blendMode);
    this.color = color;
    this.width = Math.max(1, width);
    this.hz = hz;
    this.wrap = wrap;
  }

  @Override
  public LedColor sample(int ledIndex, double nowSeconds) {
    int length = getRange().length();
    int localIndex = ledIndex - getRange().startInclusive();

    if (length <= 0) {
      return null;
    }

    if (hz <= 0) {
      return localIndex < width ? color : null;
    }

    int step = (int) Math.floor(nowSeconds * hz);

    if (wrap) {
      int head = floorMod(step, length);
      int delta = floorMod(localIndex - head, length);
      return delta >= 0 && delta < width ? color : null;
    }

    int cycleLength = length + width;
    int start = -width + 1 + floorMod(step, cycleLength);
    boolean lit = localIndex >= start && localIndex < start + width;
    return lit ? color : null;
  }

  private static int floorMod(int x, int y) {
    int mod = x % y;
    return mod < 0 ? mod + y : mod;
  }
}
