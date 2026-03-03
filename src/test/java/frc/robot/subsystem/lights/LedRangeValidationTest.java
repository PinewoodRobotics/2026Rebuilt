package frc.robot.util.lighting;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import frc.robot.constant.LEDConstants;
import org.junit.jupiter.api.Test;

class LedRangeValidationTest {
  @Test
  void rejectsInvertedRange() {
    assertThrows(IllegalArgumentException.class, () -> new LedRange(10, 9));
  }

  @Test
  void rejectsOutOfBoundsRange() {
    assertThrows(IllegalArgumentException.class, () -> new LedRange(-1, 5));
    assertThrows(IllegalArgumentException.class, () -> new LedRange(0, LEDConstants.ledEndIndex + 1));
  }

  @Test
  void acceptsSingleAndFullRange() {
    LedRange single = assertDoesNotThrow(() -> LedRange.single(25));
    LedRange full = assertDoesNotThrow(() -> new LedRange(LEDConstants.ledStartIndex, LEDConstants.ledEndIndex));

    assertEquals(25, single.startInclusive());
    assertEquals(25, single.endInclusive());
    assertEquals(LEDConstants.ledCount, full.length());
  }
}
