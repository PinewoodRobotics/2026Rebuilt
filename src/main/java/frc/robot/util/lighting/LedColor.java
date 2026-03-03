package frc.robot.util.lighting;

public record LedColor(int red, int green, int blue, int white) {
  public static final LedColor BLACK = new LedColor(0, 0, 0, 0);

  public LedColor {
    red = clamp(red);
    green = clamp(green);
    blue = clamp(blue);
    white = clamp(white);
  }

  public LedColor(int red, int green, int blue) {
    this(red, green, blue, 0);
  }

  public LedColor add(LedColor other) {
    return new LedColor(
        clamp(red + other.red),
        clamp(green + other.green),
        clamp(blue + other.blue),
        clamp(white + other.white));
  }

  public LedColor scale(double scalar) {
    double clamped = clamp01(scalar);
    return new LedColor(
        (int) Math.round(red * clamped),
        (int) Math.round(green * clamped),
        (int) Math.round(blue * clamped),
        (int) Math.round(white * clamped));
  }

  public static int clamp(int value) {
    return Math.max(0, Math.min(255, value));
  }

  public static double clamp01(double value) {
    return Math.max(0.0, Math.min(1.0, value));
  }
}
