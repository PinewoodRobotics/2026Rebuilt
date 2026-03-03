package frc.robot.util.lighting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.util.lighting.effects.ProgressBarEffect;
import frc.robot.util.lighting.effects.SolidEffect;
import java.util.Arrays;
import org.junit.jupiter.api.Test;

class ProgressBarEffectTest {
  @Test
  void fillsExpectedCountAtBoundaries() {
    LightsEngine engine = new LightsEngine(10, 10);
    EffectHandle<Double> bar = engine.addEffect(new ProgressBarEffect(
        new LedRange(0, 9),
        new LedColor(0, 255, 0),
        new LedColor(10, 10, 10),
        5,
        BlendMode.OVERWRITE));

    engine.setProgress(bar, 0.0);
    LedColor[] frame0 = materializeFrame(10, engine.render(0.0));
    assertEquals(0, countColor(frame0, new LedColor(0, 255, 0)));

    engine.setProgress(bar, 0.5);
    LedColor[] frame1 = applyDiff(frame0, engine.render(0.1));
    assertEquals(5, countColor(frame1, new LedColor(0, 255, 0)));

    engine.setProgress(bar, 1.0);
    LedColor[] frame2 = applyDiff(frame1, engine.render(0.2));
    assertEquals(10, countColor(frame2, new LedColor(0, 255, 0)));
  }

  @Test
  void overlayLeavesUnfilledPixelsUntouched() {
    LightsEngine engine = new LightsEngine(10, 10);
    engine.addEffect(new SolidEffect(new LedRange(0, 9), new LedColor(15, 15, 15), 5, BlendMode.OVERWRITE));
    EffectHandle<Double> overlay = engine.addEffect(new ProgressBarEffect(
        new LedRange(0, 9),
        new LedColor(100, 0, 255),
        null,
        10,
        BlendMode.ADD));

    engine.setProgress(overlay, 0.3);
    LedColor[] frame = materializeFrame(10, engine.render(0.0));

    for (int i = 0; i < 3; i++) {
      assertEquals(new LedColor(115, 15, 255), frame[i]);
    }
    for (int i = 3; i < 10; i++) {
      assertEquals(new LedColor(15, 15, 15), frame[i]);
    }
  }

  private static LedColor[] materializeFrame(int ledCount, LightsEngine.RenderResult result) {
    LedColor[] frame = new LedColor[ledCount];
    Arrays.fill(frame, LedColor.BLACK);
    return applyDiff(frame, result);
  }

  private static LedColor[] applyDiff(LedColor[] previousFrame, LightsEngine.RenderResult result) {
    LedColor[] next = Arrays.copyOf(previousFrame, previousFrame.length);
    for (LightsEngine.LedSegment segment : result.segments()) {
      for (int i = segment.startInclusive(); i <= segment.endInclusive(); i++) {
        next[i] = segment.color();
      }
    }
    return next;
  }

  private static int countColor(LedColor[] frame, LedColor color) {
    int count = 0;
    for (LedColor pixel : frame) {
      if (pixel.equals(color)) {
        count++;
      }
    }
    return count;
  }
}
