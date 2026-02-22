package frc.robot.subsystem;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.LEDConstants;

import java.util.HashMap;
import java.util.Map;

public class LEDSubsystem extends SubsystemBase {
  private static LEDSubsystem instance;

  public static LEDSubsystem GetInstance() {
    if (instance == null) {
      instance = new LEDSubsystem();
    }
    return instance;
  }

  private final CANdle m_candle = new CANdle(LEDConstants.candleCANId);

  private static final int LED_START = 0;
  private static final int LED_END = LEDConstants.ledEndIndex;

  private static final RGBWColor PROGRESS_DEFAULT_FILL = new RGBWColor(0, 255, 0);
  private static final RGBWColor PROGRESS_DEFAULT_EMPTY = new RGBWColor(15, 15, 15);

  private final Map<String, ProgressBar> m_progressBars = new HashMap<>();
  private boolean m_progressBarMode = false;

  public LEDSubsystem() {}

  /** Set LEDs to a solid RGB color (0-255 per channel). */
  public void setSolidColor(int r, int g, int b) {
    m_progressBarMode = false;
    m_candle.setControl(
        new SolidColor(LED_START, LED_END).withColor(new RGBWColor(r, g, b)));
  }

  /** Set LEDs to a solid color. */
  public void setSolidColor(RGBWColor color) {
    m_progressBarMode = false;
    m_candle.setControl(new SolidColor(LED_START, LED_END).withColor(color));
  }

  /** Set LEDs off (black). */
  public void setOff() {
    setSolidColor(0, 0, 0);
  }

  /**
   * Start rainbow animation across the LEDs.
   *
   * @param brightness 0.0 to 1.0
   * @param speedHz    frame rate in Hz (2-1000), higher = faster
   */
  public void setRainbow(double brightness, double speedHz) {
    m_progressBarMode = false;
    m_candle.setControl(
        new RainbowAnimation(LED_START, LED_END)
            .withBrightness(brightness)
            .withFrameRate(speedHz)
            .withDirection(AnimationDirectionValue.Forward));
  }

  /** Start rainbow animation at default brightness and speed. */
  public void setRainbow() {
    setRainbow(1.0, 50.0);
  }

  /**
   * Add a progress bar between LED indices. Progress fills left-to-right.
   *
   * @param id       unique id (e.g. "one")
   * @param startLed first LED index (inclusive)
   * @param endLed   last LED index (inclusive)
   */
  public void addProgressBar(String id, int startLed, int endLed) {
    m_progressBars.put(id, new ProgressBar(startLed, endLed, PROGRESS_DEFAULT_FILL, PROGRESS_DEFAULT_EMPTY, false));
  }

  /**
   * Add a progress bar with custom colors.
   */
  public void addProgressBar(String id, int startLed, int endLed, RGBWColor fillColor, RGBWColor emptyColor) {
    m_progressBars.put(id, new ProgressBar(startLed, endLed, fillColor, emptyColor, false));
  }

  /**
   * Add a progress bar that overlays another (same range). Only the filled portion is drawn;
   * the empty portion is left transparent so the bar below shows through.
   */
  public void addProgressBarOverlay(String id, int startLed, int endLed, RGBWColor fillColor) {
    m_progressBars.put(id, new ProgressBar(startLed, endLed, fillColor, null, true));
  }

  /** Remove a progress bar by id. */
  public void removeProgressBar(String id) {
    m_progressBars.remove(id);
  }

  /** Remove all progress bars. */
  public void clearProgressBars() {
    m_progressBars.clear();
  }

  /**
   * Set progress for a bar. 0.0 = empty, 1.0 = full.
   *
   * @param id       progress bar id
   * @param progress value in [0, 1]
   */
  public void setProgress(String id, double progress) {
    ProgressBar bar = m_progressBars.get(id);
    if (bar == null) {
      return;
    }
    bar.progress = Math.max(0, Math.min(1, progress));
    m_progressBarMode = true;
    renderProgressBars();
  }

  @Override
  public void periodic() {
    if (m_progressBarMode && !m_progressBars.isEmpty()) {
      renderProgressBars();
    }
  }

  private void renderProgressBars() {
    int ledCount = LED_END - LED_START + 1;
    int[] red = new int[ledCount];
    int[] green = new int[ledCount];
    int[] blue = new int[ledCount];
    int[] white = new int[ledCount];

    // Compose all progress bars additively so overlapping bars blend, not overwrite.
    for (ProgressBar bar : m_progressBars.values()) {
      int total = bar.endLed - bar.startLed + 1;
      int filledCount = (int) Math.round(bar.progress * total);
      filledCount = Math.max(0, Math.min(total, filledCount));

      for (int led = bar.startLed; led <= bar.endLed; led++) {
        int index = led - LED_START;
        int barIndex = led - bar.startLed;
        boolean isFilled = barIndex < filledCount;
        RGBWColor colorToAdd = null;

        if (isFilled) {
          colorToAdd = bar.fillColor;
        } else if (!bar.overlay) {
          colorToAdd = bar.emptyColor;
        }

        if (colorToAdd != null) {
          red[index] = clampColor(red[index] + colorToAdd.Red);
          green[index] = clampColor(green[index] + colorToAdd.Green);
          blue[index] = clampColor(blue[index] + colorToAdd.Blue);
          white[index] = clampColor(white[index] + colorToAdd.White);
        }
      }
    }

    // Emit contiguous ranges with identical color to minimize CAN traffic.
    int segmentStart = LED_START;
    for (int i = 1; i <= ledCount; i++) {
      boolean atEnd = i == ledCount;
      boolean changed = !atEnd && (red[i] != red[i - 1]
          || green[i] != green[i - 1]
          || blue[i] != blue[i - 1]
          || white[i] != white[i - 1]);

      if (atEnd || changed) {
        int colorIndex = i - 1;
        m_candle.setControl(new SolidColor(segmentStart, LED_START + colorIndex).withColor(
            new RGBWColor(red[colorIndex], green[colorIndex], blue[colorIndex], white[colorIndex])));
        if (!atEnd) {
          segmentStart = LED_START + i;
        }
      }
    }
  }

  private static int clampColor(int value) {
    return Math.max(0, Math.min(255, value));
  }

  private static class ProgressBar {
    final int startLed;
    final int endLed;
    final RGBWColor fillColor;
    final RGBWColor emptyColor;
    final boolean overlay;
    double progress;

    ProgressBar(int startLed, int endLed, RGBWColor fillColor, RGBWColor emptyColor, boolean overlay) {
      this.startLed = startLed;
      this.endLed = endLed;
      this.fillColor = fillColor;
      this.emptyColor = emptyColor;
      this.overlay = overlay;
      this.progress = 0;
    }
  }

  /**
   * Get the underlying CANdle for advanced control (animations, config, etc.).
   */
  public CANdle getCandle() {
    return m_candle;
  }
}
