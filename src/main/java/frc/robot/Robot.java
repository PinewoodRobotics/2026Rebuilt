package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import frc.robot.constant.CommunicationConstants;
import lombok.Getter;

public class Robot extends LoggedRobot {
  @Getter
  private static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  @Getter
  private static NetworkTable dashboard = networkTableInstance.getTable(CommunicationConstants.kSharedTable);

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  public Robot() {
    // Persist the full AdvantageKit stream to disk: "/U/logs" on a USB stick
    // when running on the RIO, "logs" folder in simulation.
    Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.onAnyModeStart();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.onAnyModeStart();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void utilityInit() {
    m_robotContainer.onAnyModeStart();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void utilityPeriodic() {
  }
}
