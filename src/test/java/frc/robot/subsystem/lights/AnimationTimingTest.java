package frc.robot.util.lighting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.util.lighting.effects.BlinkEffect;
import frc.robot.util.lighting.effects.BreatheEffect;
import frc.robot.util.lighting.effects.ChaseEffect;
import frc.robot.util.lighting.effects.RainbowEffect;
import org.junit.jupiter.api.Test;

class AnimationTimingTest {
  @Test
  void blinkIsDeterministicAcrossTime() {
    BlinkEffect blink = new BlinkEffect(
        LedRange.single(0),
        new LedColor(255, 0, 0),
        LedColor.BLACK,
        1.0,
        0,
        BlendMode.OVERWRITE);

    assertEquals(new LedColor(255, 0, 0), blink.sample(0, 0.0));
    assertEquals(LedColor.BLACK, blink.sample(0, 0.6));
    assertEquals(new LedColor(255, 0, 0), blink.sample(0, 1.1));
  }

  @Test
  void breatheChangesBrightnessDeterministically() {
    BreatheEffect breathe = new BreatheEffect(
        LedRange.single(0),
        new LedColor(200, 100, 0),
        1.0,
        0.2,
        0.8,
        0,
        BlendMode.OVERWRITE);

    LedColor c0 = breathe.sample(0, 0.0);
    LedColor cQuarter = breathe.sample(0, 0.25);

    assertNotEquals(c0, cQuarter);
    assertTrue(c0.red() >= 40 && c0.red() <= 160);
    assertEquals(new LedColor(160, 80, 0), cQuarter);
  }

  @Test
  void chaseMovesAcrossStripDeterministically() {
    ChaseEffect chase = new ChaseEffect(
        new LedRange(0, 4),
        new LedColor(0, 255, 0),
        2,
        1.0,
        false,
        0,
        BlendMode.OVERWRITE);

    assertEquals(new LedColor(0, 255, 0), chase.sample(0, 0.0));
    assertEquals(null, chase.sample(1, 0.0));

    assertEquals(new LedColor(0, 255, 0), chase.sample(0, 1.0));
    assertEquals(new LedColor(0, 255, 0), chase.sample(1, 1.0));
  }

  @Test
  void rainbowIsDeterministicForSameInputAndChangesOverTime() {
    RainbowEffect rainbow = new RainbowEffect(new LedRange(0, 9), 0.5, 1.0, 0, BlendMode.OVERWRITE);

    LedColor atT0 = rainbow.sample(3, 0.0);
    LedColor atT0Again = rainbow.sample(3, 0.0);
    LedColor atT1 = rainbow.sample(3, 1.0);

    assertEquals(atT0, atT0Again);
    assertNotEquals(atT0, atT1);
  }
}
