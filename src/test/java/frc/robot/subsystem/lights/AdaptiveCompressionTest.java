package frc.robot.util.lighting;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.util.lighting.effects.RainbowEffect;
import org.junit.jupiter.api.Test;

class AdaptiveCompressionTest {
  @Test
  void compressesWhenSegmentCountExceedsCap() {
    LightsEngine engine = new LightsEngine(40, 4);
    engine.addEffect(new RainbowEffect(new LedRange(0, 39), 25.0, 1.0, 0, BlendMode.OVERWRITE));

    LightsEngine.RenderResult result = engine.render(0.1234);

    assertTrue(result.adaptiveCompressionActive());
    assertTrue(result.segments().size() <= 4);
    assertTrue(result.changedLedCount() > 0);
  }
}
