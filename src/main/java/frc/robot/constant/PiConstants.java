package frc.robot.constant;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import lombok.AllArgsConstructor;
import pwrup.frc.core.online.raspberrypi.AutomaticPiNetwork;
import pwrup.frc.core.online.raspberrypi.PiNetwork;
import pwrup.frc.core.online.raspberrypi.WeightedProcess;

public class PiConstants {
  public static File configFilePath = new File(
      Filesystem.getDeployDirectory().getAbsolutePath() + "/config");

  @AllArgsConstructor
  public static enum ProcessType implements WeightedProcess {
    // position-extrapolator is the equivalent run definition name defined in
    // deploy.py
    POSE_EXTRAPOLATOR("position-extrapolator"), // example process name
    APRIL_TAG_DETECTOR("april-server"); // example process name

    private final String name;

    @Override
    public String toString() {
      return name;
    }

    @Override
    public double getWeight() {
      switch (this) {
        case POSE_EXTRAPOLATOR:
          return 0.5;
        case APRIL_TAG_DETECTOR:
          return 1.0;
      }

      return 0.0;
    }
  }

  // This is the time in seconds that the network will wait for the Pis to be
  // discovered and the processes to be started.
  public static int networkInitializeTimeSec = 4;

  // This network automatically finds the raspberry PIs on the robot and
  // dynamically sets up the required processes on each Pi. This assumes that all
  // process types are available on all Pis.
  public static final AutomaticPiNetwork<ProcessType> network = new AutomaticPiNetwork<ProcessType>(
      networkInitializeTimeSec /* ProcessType.APRIL_TAG_DETECTOR, ProcessType.POSE_EXTRAPOLATOR */);

  static {
    /*
     * If you need to add a constrained process to a specific Pi, you can do so
     * here. A constrained process is a process that is only allowed to run on a
     * specific Pi. For example, if you have a single process requiring a specific
     * pi feature (like a gpu), you can add it here and it will completely bypass
     * the weighting system and just be auto added to the respective pi.
     *
     * AutomaticPiNetwork.AddConstrainedProcesses(
     * new ConstrainedProcess<>(ProcessType.APRIL_TAG_DETECTOR, "tripli"));
     */
  }

  /**
   * Autobahn topic names used for subscribing/publishing
   */
  public static class AutobahnConfig {
    public static String poseSubscribeTopic = "pos-extrapolator/robot-position";
    public static String cameraTagsViewTopic = "apriltag/tag";
  }
}
