package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

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

public class Robot extends LoggedRobot {

  @Getter
  private static OptionalAutobahn communicationClient = new OptionalAutobahn();
  @Getter
  private static boolean onlineStatus;

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  /**
   * Static accessor for the Autobahn client (used by subsystems during
   * construction)
   */
  public static OptionalAutobahn getAutobahnClient() {
    return communicationClient;
  }

  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    RPC.SetClient(communicationClient);
    // PrintPiLogs.ToSystemOut(communicationClient,
    // TopicConstants.kPiTechnicalLogTopic);
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

  /**
   * Initializes the network. This is used to connect to the pi network and start
   * the processes on the pis.
   */
  private void initializeNetwork() {
    new Thread(() -> {
      PiConstants.network.initialize();
      onlineStatus = PiConstants.network.getMainPi() != null;
      if (!onlineStatus) {
        System.out.println("WARNING: NO NETWORK INITIALIZED! SOME FEATURES MAY NOT BE AVAILABLE AT THIS TIME.");
        return;
      }

      // The main Pi is defined as the first one added to the network. In essence this
      // is here to create an addr to some pi to which the robot can connect. Without
      // going into too much detail, if the robot connects to one Pi, it starts to
      // receive data from anything running on the pi network (vision/etc.)
      var address = new Address(PiConstants.network.getMainPi().getHost(), PiConstants.network.getMainPi().getPort());
      var realClient = new AutobahnClient(address); // this is the pubsub server
      realClient.begin().join(); // this essentially attempts to connect to the pi specified in the
                                 // constructor.
      communicationClient.setAutobahnClient(realClient); // set the real client to the optional autobahn client

      // Very important bit here:
      // The network has a -> shared config <- which must be sent to it on start. At
      // each pi in the network there runs a server listening to a port to which you
      // can send commands regarding the functionality of the pi (for example "start
      // [a, b, c]" or "stop [a, b, c]").
      // Anyway, these two commands 1) set the config on the pi (thereby updating the
      // pi config to your local typescript config) and 2) restart all the pi
      // processes (what this means is that the network, under the hood, sends 2
      // commands -- to stop all processes running on the pi and then to restart the
      // new selected processes)
      PiConstants.network.setConfig(readFromFile(PiConstants.configFilePath));
      boolean success = PiConstants.network.restartAllPis();
      System.out.println("INITIALIZED NETWORK!");
      if (!success) { // one of the exit codes is not successful in http req
        System.out.println("ERROR: Failed to restart Pis");
      }
    }).start();
  }

  private String readFromFile(File path) {
    try {
      return Files.readString(Paths.get(path.getAbsolutePath()));
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }
}
