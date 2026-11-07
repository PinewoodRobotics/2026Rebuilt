package frc.robot.constant;

import com.ctre.phoenix6.CANBus;

import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;

public class BotConstants {

  public static final CANPort kCanPort = CANPort.CAN_S0;
  public static final CANBus kCanBus = new CANBus(kCanPort);

  public static final Field kFieldLayout = Fields.FRC_2026_REBUILT_WELDED.loadField();

  // public static final Alliance alliance = Alliance.RED; //
  // MatchState.getAlliance().orElse(Alliance.BLUE);

  public static enum RobotVariant {
    ABOT,
    BBOT
  }

  public static final RobotVariant robotType = RobotVariant.ABOT;
  public static Alliance alliance = MatchState.getAlliance().orElse(Alliance.BLUE);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Mode currentMode = Mode.REAL;

  public static void SetAlliance() {
    alliance = MatchState.getAlliance().orElse(Alliance.BLUE);
  }
}
