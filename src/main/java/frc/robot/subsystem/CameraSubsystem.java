package frc.robot.subsystem;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.littletonrobotics.junction.Logger;

import autobahn.client.NamedCallback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constant.BotConstants;
import frc.robot.constant.CommunicationConstants;
import frc.robot.util.CustomUtil;
import lombok.AllArgsConstructor;
import lombok.Getter;
import frc4765.proto.sensor.Apriltags.ProcessedTag;
import frc4765.proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import frc4765.proto.sensor.GeneralSensorDataOuterClass.SensorName;;

public class CameraSubsystem extends SubsystemBase {

  private static CameraSubsystem self;

  private static final long kTagTimeoutMs = 200;

  /**
   * Queue of timed tags. This is fully instant concurrently because you only need
   * to retrieve the head of the queue when reading. Therefore, the writer
   * (another thread) only adds to the end of the queue. Therefore, they don't
   * contradict each other.
   */
  private volatile ConcurrentLinkedQueue<TimedTag> q = new ConcurrentLinkedQueue<>();
  private HashSet<TimedTag> localQ = new HashSet<>();

  @Getter
  @AllArgsConstructor
  private static class TimedTag {
    public ProcessedTag tag;
    public long timestamp;

    @Override
    public boolean equals(Object obj) {
      if (this == obj) {
        return true;
      }
      if (obj == null || getClass() != obj.getClass()) {
        return false;
      }
      TimedTag other = (TimedTag) obj;
      return tag.getId() == other.tag.getId();
    }

    @Override
    public int hashCode() {
      return 31 * tag.getId();
    }
  }

  public static CameraSubsystem GetInstance() {
    if (self == null) {
      self = new CameraSubsystem();
    }

    return self;
  }

  public CameraSubsystem() {
    Robot.getCommunicationClient().subscribe(CommunicationConstants.kCameraTagsViewTopic,
        NamedCallback.FromConsumer(this::subscription));
  }

  public void subscription(byte[] payload) {
    GeneralSensorData data = CustomUtil.DeserializeSilent(payload, GeneralSensorData.class);
    if (data == null || data.getSensorName() != SensorName.APRIL_TAGS)
      return;
    if (!data.getApriltags().hasWorldTags())
      return;
    long now = System.currentTimeMillis();
    for (ProcessedTag tag : data.getApriltags().getWorldTags().getTagsList()) {
      q.add(new TimedTag(tag, now));
    }
  }

  @Override
  public void periodic() {
    List<Pose2d> positionsRobot = new ArrayList<>();
    List<Pose3d> positionsReal = new ArrayList<>();

    localQ.removeIf(timedTag -> System.currentTimeMillis() - timedTag.timestamp > kTagTimeoutMs);

    TimedTag timedTag;
    while ((timedTag = q.poll()) != null) {
      localQ.add(timedTag);
    }

    for (var t : localQ) {
      var tag = t.getTag();
      int id = tag.getId();
      double confidence = tag.getConfidence();
      var posRaw = tag.getPositionWPILib();
      var rotRaw = tag.getRotationWPILib();

      Pose2d positionRobot = new Pose2d(
          (double) posRaw.getX(), (double) posRaw.getY(),
          new Rotation2d((double) rotRaw.getDirectionX().getX(), (double) rotRaw.getDirectionX().getY()));

      Pose3d positionField = BotConstants.kFieldLayout.getTagPose(id).orElse(new Pose3d());

      positionsRobot.add(positionRobot);
      positionsReal.add(positionField);

      Logger.recordOutput("Camera/Tags/Confidences/" + id, confidence);
    }

    Logger.recordOutput("Camera/Tags/PositionsRobot", positionsRobot.toArray(new Pose2d[0]));
    Logger.recordOutput("Camera/Tags/PositionsField", positionsReal.toArray(new Pose3d[0]));
  }
}
