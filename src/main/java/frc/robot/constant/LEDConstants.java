package frc.robot.constant;

public class LEDConstants {
  private LEDConstants() {
  }

  /** CAN ID of the CANdle device. Configure in Phoenix Tuner. */
  public static final int candleCANId = 26;

  /** Full LED address space on CANdle: onboard [0..7] + strip [8..399]. */
  public static final int ledStartIndex = 0;
  public static final int ledEndIndex = 399;
  public static final int ledCount = ledEndIndex - ledStartIndex + 1;

  /** Maximum number of contiguous solid-color writes per periodic cycle. */
  public static final int maxSolidWritesPerCycle = 48;

  /** Foundational named zones. */
  public static final int onboardStartIndex = 0;
  public static final int onboardEndIndex = 7;

  public static final int externalStripStartIndex = 8;
  public static final int externalStripEndIndex = 399;

  public static final int leftHalfStartIndex = 8;
  public static final int leftHalfEndIndex = 203;

  public static final int rightHalfStartIndex = 204;
  public static final int rightHalfEndIndex = 399;
}
