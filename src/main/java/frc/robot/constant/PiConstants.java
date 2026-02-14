package frc.robot.constant;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

public class PiConstants {
  public static File configFilePath = new File(
      Filesystem.getDeployDirectory().getAbsolutePath() + "/config");

  public static int networkInitializeTimeSec = 4;
}
