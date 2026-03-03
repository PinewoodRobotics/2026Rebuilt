package frc.robot.util.lighting;

import frc.robot.constant.LEDConstants;

public enum LightZone {
  ONBOARD(LEDConstants.onboardStartIndex, LEDConstants.onboardEndIndex),
  FULL_STRIP(LEDConstants.ledStartIndex, LEDConstants.ledEndIndex),
  EXTERNAL_STRIP(LEDConstants.externalStripStartIndex, LEDConstants.externalStripEndIndex),
  LEFT_HALF(LEDConstants.leftHalfStartIndex, LEDConstants.leftHalfEndIndex),
  RIGHT_HALF(LEDConstants.rightHalfStartIndex, LEDConstants.rightHalfEndIndex);

  private final int startInclusive;
  private final int endInclusive;

  LightZone(int startInclusive, int endInclusive) {
    this.startInclusive = startInclusive;
    this.endInclusive = endInclusive;
  }

  public LedRange range() {
    return new LedRange(startInclusive, endInclusive);
  }
}
