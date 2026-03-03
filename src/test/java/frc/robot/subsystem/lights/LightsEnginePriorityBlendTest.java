package frc.robot.util.lighting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.util.lighting.effects.SolidEffect;
import java.util.Arrays;
import org.junit.jupiter.api.Test;

class LightsEnginePriorityBlendTest {
  @Test
  void higherPriorityOverwriteWinsInOverlap() {
    LightsEngine engine = new LightsEngine(10, 10);

    engine.addEffect(new SolidEffect(new LedRange(0, 9), new LedColor(255, 0, 0), 5, BlendMode.OVERWRITE));
    engine.addEffect(new SolidEffect(new LedRange(3, 6), new LedColor(0, 0, 255), 10, BlendMode.OVERWRITE));

    LightsEngine.RenderResult result = engine.render(0.0);
    LedColor[] frame = materializeFrame(10, result);

    for (int i = 0; i <= 2; i++) {
      assertEquals(new LedColor(255, 0, 0), frame[i]);
    }
    for (int i = 3; i <= 6; i++) {
      assertEquals(new LedColor(0, 0, 255), frame[i]);
    }
    for (int i = 7; i <= 9; i++) {
      assertEquals(new LedColor(255, 0, 0), frame[i]);
    }
  }

  @Test
  void additiveBlendClampsChannels() {
    LightsEngine engine = new LightsEngine(5, 10);

    engine.addEffect(new SolidEffect(new LedRange(0, 4), new LedColor(200, 0, 0), 5, BlendMode.OVERWRITE));
    engine.addEffect(new SolidEffect(new LedRange(0, 4), new LedColor(100, 0, 0), 10, BlendMode.ADD));

    LightsEngine.RenderResult result = engine.render(0.0);
    LedColor[] frame = materializeFrame(5, result);

    for (LedColor pixel : frame) {
      assertEquals(new LedColor(255, 0, 0), pixel);
    }
  }

  private static LedColor[] materializeFrame(int ledCount, LightsEngine.RenderResult result) {
    LedColor[] frame = new LedColor[ledCount];
    Arrays.fill(frame, LedColor.BLACK);
    for (LightsEngine.LedSegment segment : result.segments()) {
      for (int i = segment.startInclusive(); i <= segment.endInclusive(); i++) {
        frame[i] = segment.color();
      }
    }
    return frame;
  }
}
