package frc.robot.constant;

public class HardwareConstants {
  public record PigeonConfig(int canId, double mountPoseYawDeg, double mountPosePitchDeg, double mountPoseRollDeg) {
  }

  public static final PigeonConfig[] kPigeonConfigs = {
      new PigeonConfig(40, 0.0, 0.0, 0.0),
      new PigeonConfig(41, 0.0, 0.0, 0.0),
  };

  public enum RobotMainGyro {
    GlobalClosest,
    One,
    Two,
  }

  public static final RobotMainGyro kRobotMainGyro = RobotMainGyro.Two;
}
