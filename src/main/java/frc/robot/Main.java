package frc.robot;

import org.wpilib.framework.RobotBase;

public final class Main {

  private Main() {
  }

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
