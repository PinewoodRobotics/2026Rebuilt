package frc.robot.util.lighting.effects;

import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightEffect;

public class RainbowEffect extends LightEffect<Void> {
  private final double hz;
  private final double brightness;

  public RainbowEffect(LedRange range, double hz, double brightness, int priority, BlendMode blendMode) {
    super(range, priority, blendMode);
    this.hz = hz;
    this.brightness = LedColor.clamp01(brightness);
  }

  @Override
  public LedColor sample(int ledIndex, double nowSeconds) {
    int length = getRange().length();
    int localIndex = ledIndex - getRange().startInclusive();

    double hue = (localIndex / (double) Math.max(1, length)) + (nowSeconds * hz);
    hue = hue - Math.floor(hue);

    return hsvToRgb(hue, 1.0, brightness);
  }

  private static LedColor hsvToRgb(double h, double s, double v) {
    double chroma = v * s;
    double hPrime = h * 6.0;
    double x = chroma * (1.0 - Math.abs((hPrime % 2.0) - 1.0));

    double r1;
    double g1;
    double b1;

    if (hPrime < 1.0) {
      r1 = chroma;
      g1 = x;
      b1 = 0;
    } else if (hPrime < 2.0) {
      r1 = x;
      g1 = chroma;
      b1 = 0;
    } else if (hPrime < 3.0) {
      r1 = 0;
      g1 = chroma;
      b1 = x;
    } else if (hPrime < 4.0) {
      r1 = 0;
      g1 = x;
      b1 = chroma;
    } else if (hPrime < 5.0) {
      r1 = x;
      g1 = 0;
      b1 = chroma;
    } else {
      r1 = chroma;
      g1 = 0;
      b1 = x;
    }

    double m = v - chroma;
    int r = (int) Math.round((r1 + m) * 255.0);
    int g = (int) Math.round((g1 + m) * 255.0);
    int b = (int) Math.round((b1 + m) * 255.0);
    return new LedColor(r, g, b, 0);
  }
}
