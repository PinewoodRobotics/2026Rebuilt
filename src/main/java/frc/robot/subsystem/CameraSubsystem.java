package frc.robot.subsystem;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.littletonrobotics.junction.Logger;

import autobahn.client.NamedCallback;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constant.PiConstants;
import frc.robot.constant.TopicConstants;
import frc.robot.util.CustomUtil;
import lombok.AllArgsConstructor;
import lombok.Getter;
import frc4765.proto.sensor.Apriltags.AprilTagData;
import frc4765.proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import frc4765.proto.sensor.GeneralSensorDataOuterClass.SensorName;;

public class CameraSubsystem extends SubsystemBase {

  private static CameraSubsystem self;

  /**
   * Queue of timed tags. This is fully instant concurrently because you only need
   * to retrieve the head of the queue when reading. Therefore, the writer
   * (another thread) only adds to the end of the queue. Therefore, they don't
   * contradict each other.
   */
  private final ConcurrentLinkedQueue<TimedTags> q = new ConcurrentLinkedQueue<>();

  private static final String FIELD_LAYOUT_DEPLOY_FILE = "2026-rebuilt-welded.json";
  private static final AprilTagFieldLayout FIELD_LAYOUT = loadFieldLayout();

  /**
   * Load the field layout from the deploy file.
   * 
   * @return The field layout.
   * @throws IOException If the field layout cannot be loaded.
   *                     TODO: figure out why the default way does not work.
   * 
   *                     How it is meant to work:
   *                     AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
   */
  private static AprilTagFieldLayout loadFieldLayout() {
    var path = Filesystem.getDeployDirectory().toPath().resolve(FIELD_LAYOUT_DEPLOY_FILE);
    try {
      return AprilTagFieldLayout.loadFromResource(path.toString());
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }

  @Getter
  @AllArgsConstructor
  private static class TimedTags {
    public AprilTagData tags;
    public long timestamp;
  }

  public static CameraSubsystem GetInstance() {
    if (self == null) {
      self = new CameraSubsystem();
    }

    return self;
  }

  public CameraSubsystem() {
    Robot.getCommunicationClient().subscribe(TopicConstants.kCameraTagsViewTopic,
        NamedCallback.FromConsumer(this::subscription));
  }

  public void subscription(byte[] payload) {
    GeneralSensorData data = CustomUtil.DeserializeSilent(payload, GeneralSensorData.class);
    if (data == null || data.getSensorName() != SensorName.APRIL_TAGS)
      return;

    q.add(new TimedTags(data.getApriltags(), System.currentTimeMillis()));
  }

  @Override
  public void periodic() {
    List<Pose2d> positionsRobot = new ArrayList<>();
    List<Pose3d> positionsReal = new ArrayList<>();

    TimedTags timedTags;
    while ((timedTags = q.poll()) != null) {
      for (var tag : timedTags.getTags().getWorldTags().getTagsList()) {
        int id = tag.getId();
        double confidence = tag.getConfidence();
        var posRaw = tag.getPositionWPILib();
        var rotRaw = tag.getRotationWPILib();

        Pose2d positionRobot = new Pose2d(
            (double) posRaw.getX(), (double) posRaw.getY(),
            new Rotation2d((double) rotRaw.getDirectionX().getX(), (double) rotRaw.getDirectionX().getY()));

        Pose3d positionField = FIELD_LAYOUT.getTagPose(id).orElse(new Pose3d());

        positionsRobot.add(positionRobot);
        positionsReal.add(positionField);

        Logger.recordOutput("Camera/Tags/" + id + "/Confidence", confidence);
      }
    }

    Logger.recordOutput("Camera/Tags/PositionsRobot", positionsRobot.toArray(new Pose2d[0]));
    Logger.recordOutput("Camera/Tags/PositionsField", positionsReal.toArray(new Pose3d[0]));
  }
}
