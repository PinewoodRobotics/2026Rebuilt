package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import autobahn.client.Address;
import autobahn.client.AutobahnClient;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constant.PiConstants;
import frc.robot.util.RPC;
import lombok.Getter;
import pwrup.frc.core.online.raspberrypi.OptionalAutobahn;
import pwrup.frc.core.online.raspberrypi.discovery.PiDiscoveryUtil;
import pwrup.frc.core.online.raspberrypi.discovery.PiInfo;

public class Robot extends LoggedRobot {

  @Getter
  private static OptionalAutobahn communicationClient = new OptionalAutobahn();
  @Getter
  private static boolean onlineStatus;

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  public static OptionalAutobahn getAutobahnClient() {
    return communicationClient;
  }

  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    RPC.SetClient(communicationClient);
  }

  @Override
  public void robotInit() {
    initializeNetwork();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Logger.recordOutput("Autobahn/Connected", communicationClient.isConnected() && onlineStatus);
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
    } else {
      System.out.println("WARNING: getAutonomousCommand() returned null; nothing scheduled for auton.");
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

  private void initializeNetwork() {
    new Thread(() -> {
      List<PiInfo> pisFound = new ArrayList<>();

      try {
        pisFound = PiDiscoveryUtil.discover(PiConstants.networkInitializeTimeSec);
      } catch (IOException | InterruptedException e) {
        e.printStackTrace();
      }

      var mainPi = pisFound.get(0);

      System.out.println(mainPi);

      var address = new Address(mainPi.getHostnameLocal(), mainPi.getAutobahnPort().get());
      System.out.println(address);
      var realClient = new AutobahnClient(address);
      realClient.begin().join();
      communicationClient.setAutobahnClient(realClient);
      onlineStatus = true;
      System.out.println("SET UP CLIENT");
    }).start();
  }
}
