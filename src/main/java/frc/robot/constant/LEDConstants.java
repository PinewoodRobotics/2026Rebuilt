package frc.robot.constant;

public class LEDConstants {
  private LEDConstants() {
  }

  /** CAN ID of the CANdle device. Configure in Phoenix Tuner. */
  public static final int candleCANId = 26;

  /** Full LED address space on CANdle: onboard [0..7] + strip [8..399]. */
  public static final int ledStartIndex = 0;
  public static final int ledEndIndex = 191;
  public static final int ledCount = ledEndIndex - ledStartIndex + 1;

  /** Maximum number of contiguous solid-color writes per periodic cycle. */
  public static final int maxSolidWritesPerCycle = 48;

  /** Foundational named zones. */
  public static final int onboardStartIndex = 0;
  public static final int onboardEndIndex = 7;
}
