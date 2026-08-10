package frc.robot.util.lighting.effects;

import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightEffect;

/**
 * Larson Scanner (Knight Rider / Cylon): a single "eye" that moves back and forth
 * along the strip, with an optional trailing fade.
 */
public class LarsonScannerEffect extends LightEffect<Void> {
  private final LedColor color;
  private final int eyeWidth;
  private final int trailLength;
  private final double hz;

  public LarsonScannerEffect(
      LedRange range,
      LedColor color,
      int eyeWidth,
      int trailLength,
      double hz,
      int priority,
      BlendMode blendMode) {
    super(range, priority, blendMode);
    this.color = color;
    this.eyeWidth = Math.max(1, eyeWidth);
    this.trailLength = Math.max(0, trailLength);
    this.hz = hz;
  }

  @Override
  public LedColor sample(int ledIndex, double nowSeconds) {
    int length = getRange().length();
    int localIndex = ledIndex - getRange().startInclusive();

    if (length <= 0) {
      return null;
    }

    if (hz <= 0) {
      // Static: center at start, eye + trail only there
      int halfEye = eyeWidth / 2;
      int dist = Math.abs(localIndex - halfEye);
      return sampleBrightness(dist);
    }

    // Triangle wave: 0 -> 1 -> 0 over one period
    double phase = (nowSeconds * hz) % 1.0;
    if (phase < 0) {
      phase += 1.0;
    }
    double pos01 = phase < 0.5 ? 2.0 * phase : 2.0 * (1.0 - phase);
    double center = pos01 * (length - 1);

    double dist = Math.abs(localIndex - center);
    return sampleBrightness(dist);
  }

  private LedColor sampleBrightness(double distanceFromCenter) {
    int halfEye = eyeWidth / 2;
    if (distanceFromCenter <= halfEye) {
      return color;
    }
    int fadeStart = halfEye;
    int fadeEnd = halfEye + trailLength;
    if (trailLength <= 0 || distanceFromCenter >= fadeEnd) {
      return null;
    }
    double fade01 = 1.0 - (distanceFromCenter - fadeStart) / trailLength;
    fade01 = LedColor.clamp01(fade01);
    return color.scale(fade01);
  }
}
