package frc.robot;

import java.io.IOException;
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
import pwrup.frc.core.constant.RaspberryPiConstants;
import pwrup.frc.core.online.raspberrypi.OptionalAutobahn;
import pwrup.frc.core.online.raspberrypi.discovery.PiDiscoveryUtil;
import pwrup.frc.core.online.raspberrypi.discovery.PiInfo;

public class Robot extends LoggedRobot {
  private static final int NETWORK_RETRY_TICKS = 50;

  @Getter
  private static OptionalAutobahn communicationClient = new OptionalAutobahn();

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  private int retryCounter = 0;
  private volatile boolean networkAttemptInProgress = false;

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

    boolean currentlyConnected = communicationClient.isConnected();
    Logger.recordOutput("Autobahn/Connected", currentlyConnected);

    if (currentlyConnected || networkAttemptInProgress) {
      return;
    }

    retryCounter = (retryCounter + 1) % NETWORK_RETRY_TICKS;
    if (retryCounter == 0) {
      initializeNetwork();
    }
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
    if (networkAttemptInProgress || communicationClient.isConnected()) {
      return;
    }

    networkAttemptInProgress = true;
    new Thread(() -> {
      try {
        List<PiInfo> pisFound = PiDiscoveryUtil.discover(PiConstants.networkInitializeTimeSec);

        for (PiInfo discoveredPi : pisFound) {
          String host = discoveredPi.getHostnameLocal();
          if (host == null || host.isBlank()) {
            host = discoveredPi.getHostname();
          }
          if (host == null || host.isBlank()) {
            continue;
          }
          if (host.endsWith(".")) {
            host = host.substring(0, host.length() - 1);
          }

          int autobahnPort = discoveredPi.getAutobahnPort().orElse(RaspberryPiConstants.DEFAULT_PORT_AUTOB);
          var address = new Address(host, autobahnPort);

          try {
            var realClient = new AutobahnClient(address);
            realClient.begin().join();
            communicationClient.setAutobahnClient(realClient);
            retryCounter = 0;
            System.out.println("[PiConnect] Connected to Pi Autobahn at " + address);
            return;
          } catch (RuntimeException ignored) {
          }
        }
      } catch (IOException | InterruptedException e) {
        if (e instanceof InterruptedException) {
          Thread.currentThread().interrupt();
        }
      } finally {
        networkAttemptInProgress = false;
      }
    }, "pi-network-init").start();
  }
}
