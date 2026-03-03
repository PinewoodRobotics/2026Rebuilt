package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import autobahn.client.Address;
import autobahn.client.AutobahnClient;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.RPC;
import lombok.Getter;
import pwrup.frc.core.constant.RaspberryPiConstants;
import pwrup.frc.core.online.raspberrypi.OptionalAutobahn;
import pwrup.frc.core.online.raspberrypi.discovery.PiDiscoveryUtil;
import pwrup.frc.core.online.raspberrypi.discovery.PiInfo;

public class Robot extends LoggedRobot {
  private static final int kNetworkRetryTicks = 80;

  @Getter
  private static OptionalAutobahn communicationClient = new OptionalAutobahn();
  @Getter
  private static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

  private int retryCounter;
  private volatile boolean networkAttemptInProgress;

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
    this.networkAttemptInProgress = false;
    this.retryCounter = 0;

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

    retryCounter = (retryCounter + 1) % kNetworkRetryTicks;
    if (!currentlyConnected && !networkAttemptInProgress && retryCounter == 0) {
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
        List<PiInfo> pisFound = PiDiscoveryUtil.discover(4);
        var pi = pisFound.get(0);
        var address = new Address(pi.getHostnameLocal(),
            pi.getAutobahnPort().orElse(RaspberryPiConstants.DEFAULT_PORT_AUTOB));
        var realClient = new AutobahnClient(address);
        realClient.begin().join();

        communicationClient.setAutobahnClient(realClient);
        retryCounter = 0;

        System.out.println("[PiConnect] Connected to Pi Autobahn at " + address);
      } catch (IOException | InterruptedException e) {
        // e.printStackTrace();
      } finally {
        networkAttemptInProgress = false;
      }
    }).start();
  }
}
