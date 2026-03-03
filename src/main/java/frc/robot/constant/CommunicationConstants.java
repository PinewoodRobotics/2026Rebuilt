package frc.robot.constant;

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Robot;

public class CommunicationConstants {
  public static final String kPoseSubscribeTopic = "pos-extrapolator/robot-position";
  public static final String kPiTechnicalLogTopic = "pi-technical-log";
  public static final String kOdometryPublishTopic = "robot/odometry";
  public static final String kCameraViewTopic = "apriltag/camera";
  public static final String kCameraTagsViewTopic = "apriltag/tag";

  public static final String kSharedTable = "Shared";
  public static final String kAutoSelectRequestTopic = "PathPlanner/SelectedPath/Request";
  public static final String kAutoSelectStateTopic = "PathPlanner/SelectedPath/State";
  public static final String kOdometrySensorId = "odom";

  public static final NetworkTable kDashboardTable = Robot.getNetworkTableInstance().getTable(kSharedTable);
}
