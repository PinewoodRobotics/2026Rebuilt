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
  @Getter
  private static volatile boolean onlineStatus;

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  private int retryCounter = 0;
  private int networkAttemptIndex = 0;
  private volatile boolean connectedToPis = false;
  private volatile boolean networkAttemptInProgress = false;

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

    boolean currentlyConnected = communicationClient.isConnected() && onlineStatus;
    Logger.recordOutput("Autobahn/Connected", currentlyConnected);

    if (!currentlyConnected && connectedToPis) {
      connectedToPis = false;
      onlineStatus = false;
      System.out.println("Lost Pi connection. Will continue retrying.");
    }

    if (!connectedToPis && !networkAttemptInProgress) {
      retryCounter = (retryCounter + 1) % NETWORK_RETRY_TICKS;
      if (retryCounter == 1) {
        System.out.println(
            "[PiConnect] Waiting to start attempt #" + (networkAttemptIndex + 1) + " (in ~1 second)");
      }
      if (retryCounter == 0) {
        initializeNetwork();
      }
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
    if (connectedToPis || networkAttemptInProgress) {
      return;
    }

    int attemptNumber = ++networkAttemptIndex;
    networkAttemptInProgress = true;
    new Thread(() -> {
      try {
        System.out.println("[PiConnect #" + attemptNumber + "] Discovery started...");
        List<PiInfo> pisFound = PiDiscoveryUtil.discover(PiConstants.networkInitializeTimeSec);
        System.out.println("[PiConnect #" + attemptNumber + "] Discovery complete. Found " + pisFound.size() + " Pi(s).");
        if (pisFound.isEmpty()) {
          System.out.println("[PiConnect #" + attemptNumber + "] No Pis discovered yet. Retrying...");
          return;
        }

        for (PiInfo discoveredPi : pisFound) {
          var hostToConnect = resolvePiHost(discoveredPi);
          if (hostToConnect == null) {
            System.out.println("[PiConnect #" + attemptNumber + "] Skipping Pi with missing host info: " + discoveredPi);
            continue;
          }

          int autobahnPort = discoveredPi.getAutobahnPort().orElse(RaspberryPiConstants.DEFAULT_PORT_AUTOB);
          var address = new Address(hostToConnect, autobahnPort);

          try {
            var realClient = new AutobahnClient(address);
            realClient.begin().join();
            communicationClient.setAutobahnClient(realClient);
            connectedToPis = true;
            onlineStatus = true;
            retryCounter = 0;
            System.out.println("[PiConnect #" + attemptNumber + "] Connected to Pi Autobahn at " + address);
            return;
          } catch (RuntimeException e) {
            System.out.println(
                "[PiConnect #" + attemptNumber + "] Failed to connect to discovered Pi at " + address
                    + ". Trying next Pi...");
          }
        }

        connectedToPis = false;
        onlineStatus = false;
        System.out.println(
            "[PiConnect #" + attemptNumber + "] Discovered Pis but could not connect to any Autobahn endpoint. Retrying...");
      } catch (IOException | InterruptedException e) {
        if (e instanceof InterruptedException) {
          Thread.currentThread().interrupt();
        }
        connectedToPis = false;
        onlineStatus = false;
        System.out.println("[PiConnect #" + attemptNumber + "] Pi discovery failed. Will retry: " + e.getMessage());
      } catch (RuntimeException e) {
        connectedToPis = false;
        onlineStatus = false;
        System.out.println(
            "[PiConnect #" + attemptNumber + "] Failed to connect to Pi Autobahn. Will retry: " + e.getMessage());
      } finally {
        networkAttemptInProgress = false;
      }
    }, "pi-network-init").start();
  }

  private String resolvePiHost(PiInfo piInfo) {
    String hostnameLocal = normalizeHost(piInfo.getHostnameLocal());
    if (hostnameLocal != null) {
      return hostnameLocal;
    }

    String hostname = normalizeHost(piInfo.getHostname());
    if (hostname != null) {
      return hostname.contains(".") ? hostname : hostname + ".local";
    }

    return null;
  }

  private String normalizeHost(String host) {
    if (host == null) {
      return null;
    }

    String normalized = host.trim();
    if (normalized.isEmpty()) {
      return null;
    }

    if (normalized.endsWith(".")) {
      normalized = normalized.substring(0, normalized.length() - 1);
    }

    return normalized.isEmpty() ? null : normalized;
  }
}
