package frc.robot.util.lighting;

import frc.robot.constant.LEDConstants;

public record LedRange(int startInclusive, int endInclusive) {
  public LedRange {
    if (startInclusive > endInclusive) {
      throw new IllegalArgumentException(
          "LED range start must be <= end. Got start=" + startInclusive + ", end=" + endInclusive);
    }
    if (startInclusive < LEDConstants.ledStartIndex || endInclusive > LEDConstants.ledEndIndex) {
      throw new IllegalArgumentException(
          "LED range must be within [" + LEDConstants.ledStartIndex + ", " + LEDConstants.ledEndIndex +
              "]. Got [" + startInclusive + ", " + endInclusive + "]");
    }
  }

  public static LedRange single(int index) {
    return new LedRange(index, index);
  }

  public int length() {
    return endInclusive - startInclusive + 1;
  }

  public boolean contains(int index) {
    return index >= startInclusive && index <= endInclusive;
  }
}
