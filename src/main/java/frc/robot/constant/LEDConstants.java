package frc.robot.constant;

public class LEDConstants {
  /** CAN ID of the CANdle device. Configure in Phoenix Tuner. */
  public static final int candleCANId = 0;

  /**
   * Last LED index (inclusive). Indices 0-7 are the CANdle's onboard LEDs;
   * 8-399 are an attached addressable strip. Set to 7 for onboard only.
   */
  public static final int ledEndIndex = 67;
}
