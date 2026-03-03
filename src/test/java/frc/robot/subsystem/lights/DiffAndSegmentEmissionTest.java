package frc.robot.util.lighting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.util.lighting.effects.ProgressBarEffect;
import frc.robot.util.lighting.effects.SolidEffect;
import org.junit.jupiter.api.Test;

class DiffAndSegmentEmissionTest {
  @Test
  void unchangedFrameEmitsNoWrites() {
    LightsEngine engine = new LightsEngine(10, 10);
    engine.addEffect(new SolidEffect(new LedRange(0, 9), new LedColor(0, 0, 255), 0, BlendMode.OVERWRITE));

    LightsEngine.RenderResult first = engine.render(0.0);
    LightsEngine.RenderResult second = engine.render(0.02);

    assertTrue(first.hasWrites());
    assertEquals(1, first.segments().size());
    assertTrue(second.segments().isEmpty());
    assertEquals(0, second.changedLedCount());
  }

  @Test
  void changedFrameEmitsMinimalContiguousSegments() {
    LightsEngine engine = new LightsEngine(10, 10);
    EffectHandle<Double> progress = engine.addEffect(new ProgressBarEffect(
        new LedRange(0, 9),
        new LedColor(255, 0, 0),
        LedColor.BLACK,
        0,
        BlendMode.OVERWRITE));

    engine.setProgress(progress, 0.2);
    LightsEngine.RenderResult first = engine.render(0.0);
    assertEquals(1, first.segments().size());
    assertEquals(0, first.segments().get(0).startInclusive());
    assertEquals(1, first.segments().get(0).endInclusive());

    engine.setProgress(progress, 0.5);
    LightsEngine.RenderResult second = engine.render(0.1);

    assertEquals(1, second.segments().size());
    assertEquals(2, second.segments().get(0).startInclusive());
    assertEquals(4, second.segments().get(0).endInclusive());
  }
}
