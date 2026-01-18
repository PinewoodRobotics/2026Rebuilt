package frc.robot.constant;

import edu.wpi.first.wpilibj.RobotBase;

public class BotConstants {
  public static enum RobotVariant {
    ABOT,
    BBOT
  }

  public static final RobotVariant robotType = RobotVariant.BBOT;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Mode simMode = Mode.REAL;
  public static final Mode currentMode = Mode.REAL;
}
