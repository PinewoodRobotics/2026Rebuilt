package frc.robot.constant;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

public class PiConstants {
  public static File configFilePath = new File(
      Filesystem.getDeployDirectory().getAbsolutePath() + "/config");

  public static int networkInitializeTimeSec = 4;

  /**
   * Autobahn topic names used for subscribing/publishing
   */
  public static class AutobahnConfig {
    public static String poseSubscribeTopic = "pos-extrapolator/robot-position";
    public static String cameraTagsViewTopic = "apriltag/tag";
  }
}
