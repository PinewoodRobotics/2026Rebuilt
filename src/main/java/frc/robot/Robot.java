package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
  }

  @Override
  public void robotInit() {
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
  public void testInit() {
    m_robotContainer.onAnyModeStart();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
