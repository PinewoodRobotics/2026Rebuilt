package frc.robot.subsystem;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import org.littletonrobotics.junction.Logger;

import com.google.protobuf.InvalidProtocolBufferException;

import autobahn.client.NamedCallback;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constant.PiConstants;
import proto.sensor.Apriltags.ProcessedTag;
import proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import proto.sensor.GeneralSensorDataOuterClass.SensorName;

public class CameraSubsystem extends SubsystemBase {
  private static CameraSubsystem instance;

  private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private static class CameraMetrics {
    volatile GeneralSensorData lastSensorData;
    volatile double lastProcessingTimeMs;
    volatile double lastFps;
    volatile long lastArrivalTimeMs;
  }

  // Metrics and last packets for each camera (keyed by sensor_id from
  // GeneralSensorData).
  private final Map<String, CameraMetrics> metricsBySensor = new ConcurrentHashMap<>();

  public static CameraSubsystem GetInstance() {
    if (instance == null) {
      instance = new CameraSubsystem();
    }
    return instance;
  }

  private CameraSubsystem() {
    Robot.getAutobahnClient().subscribe(
        PiConstants.AutobahnConfig.cameraTagsViewTopic,
        NamedCallback.FromConsumer(this::subscription));
  }

  /**
   * Subscription callback for april tag data coming from the APRIL_SERVER
   * backend.
   * <p>
   * The payload is a serialized {@link GeneralSensorData} message with
   * {@code sensor_name == APRIL_TAGS}
   * and {@code apriltags.world_tags.tags} populated.
   */
  public void subscription(byte[] payload) {
    try {
      var data = GeneralSensorData.parseFrom(payload);

      if (data.getSensorName() != SensorName.APRIL_TAGS) {
        return;
      }

      String sensorId = data.getSensorId();
      if (sensorId == null || sensorId.isEmpty()) {
        sensorId = "unknown";
      }

      CameraMetrics metrics = metricsBySensor.computeIfAbsent(sensorId, id -> new CameraMetrics());

      // Compute FPS based on arrival time at the robot (no reliance on remote
      // timestamps).
      long now = System.currentTimeMillis();
      if (metrics.lastArrivalTimeMs > 0 && now > metrics.lastArrivalTimeMs) {
        long delta = now - metrics.lastArrivalTimeMs;
        metrics.lastFps = 1000.0 / delta;
      }
      metrics.lastArrivalTimeMs = now;

      // Cache processing time and last packet.
      metrics.lastProcessingTimeMs = data.getProcessingTimeMs();
      metrics.lastSensorData = data;

      var apriltags = data.getApriltags();
      var worldTags = apriltags.getWorldTags();
      var tags = worldTags.getTagsList();

      logTagMetrics(sensorId, tags);
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }

  private void logTagMetrics(String sensorId, List<ProcessedTag> tags) {
    String prefix = "Camera/" + sensorId + "/AprilTags/";

    CameraMetrics metrics = metricsBySensor.get(sensorId);
    if (metrics != null) {
      Logger.recordOutput(prefix + "frameProcessingMs", metrics.lastProcessingTimeMs);
      Logger.recordOutput(prefix + "fps", metrics.lastFps);
    }
    Logger.recordOutput(prefix + "tagCount", tags.size());

    if (tags.isEmpty()) {
      return;
    }

    for (ProcessedTag tag : tags) {
      int id = tag.getId();

      var pos = tag.getPositionWPILib();
      var rot = tag.getRotationWPILib();

      Translation3d cameraTranslation = new Translation3d(
          pos.getX(),
          pos.getY(),
          pos.getZ());

      Rotation3d cameraRotation = new Rotation3d(0.0, 0.0, rot.getYaw());

      Pose3d cameraRelativePose = new Pose3d(cameraTranslation, cameraRotation);

      Logger.recordOutput(
          prefix + "CameraRelativePose3d/Tag" + id,
          cameraRelativePose);

      FIELD_LAYOUT.getTagPose(id).ifPresent(fieldPose -> Logger.recordOutput(
          prefix + "FieldPose/Tag" + id,
          fieldPose));
    }
  }

  /**
   * Returns the last {@link GeneralSensorData} packet received for APRIL_TAGS for
   * the given sensorId, or
   * {@code null}
   * if nothing has been received yet.
   */
  public GeneralSensorData getLastSensorData(String sensorId) {
    CameraMetrics metrics = metricsBySensor.get(sensorId);
    return metrics != null ? metrics.lastSensorData : null;
  }

  /**
   * Returns whether we have ever received any april tag data for the given
   * sensorId.
   */
  public boolean hasData(String sensorId) {
    CameraMetrics metrics = metricsBySensor.get(sensorId);
    return metrics != null && metrics.lastSensorData != null;
  }

  /**
   * Returns the set of sensor IDs (camera names) that have reported APRIL_TAGS
   * data.
   */
  public Set<String> getKnownSensors() {
    return metricsBySensor.keySet();
  }

  @Override
  public void periodic() {
    double globalFps = 0.0;

    List<Pose2d> cameraRelativePoses = new ArrayList<>();
    Map<Integer, Pose2d> fieldPosesByTagId = new LinkedHashMap<>();

    long now = System.currentTimeMillis();

    for (Map.Entry<String, CameraMetrics> entry : metricsBySensor.entrySet()) {
      String sensorId = entry.getKey();
      CameraMetrics metrics = entry.getValue();
      String prefix = "Camera/" + sensorId + "/AprilTags/";
      Logger.recordOutput(prefix + "frameProcessingMs", metrics.lastProcessingTimeMs);
      Logger.recordOutput(prefix + "fps", metrics.lastFps);

      globalFps += metrics.lastFps;

      long ageMs = metrics.lastArrivalTimeMs > 0 ? now - metrics.lastArrivalTimeMs : 0;
      double estimatedLatencyMs = ageMs + metrics.lastProcessingTimeMs;

      // Age of the last frame seen from this camera and an approximate end-to-end
      // latency (capture -> backend processing -> transport -> robot use),
      // computed without trusting any remote timestamps.
      Logger.recordOutput(prefix + "frameAgeMs", ageMs);
      Logger.recordOutput(prefix + "estimatedLatencyMs", estimatedLatencyMs);

      GeneralSensorData data = metrics.lastSensorData;
      if (data == null) {
        continue;
      }

      var apriltags = data.getApriltags();
      var worldTags = apriltags.getWorldTags();
      List<ProcessedTag> tags = worldTags.getTagsList();

      for (ProcessedTag tag : tags) {
        int id = tag.getId();

        var pos = tag.getPositionWPILib();
        var rot = tag.getRotationWPILib();

        Pose2d cameraRelativePose2d = new Pose2d(
            new Translation2d(pos.getX(), pos.getY()),
            new Rotation2d(rot.getYaw()));

        cameraRelativePoses.add(cameraRelativePose2d);

        FIELD_LAYOUT.getTagPose(id)
            .ifPresent(fieldPose3d -> fieldPosesByTagId.put(id, fieldPose3d.toPose2d()));
      }
    }

    Logger.recordOutput("Camera/Global/AprilTags/fpsCombined", globalFps);

    Logger.recordOutput(
        "Camera/Global/AprilTags/CameraRelativePose2d",
        cameraRelativePoses.toArray(new Pose2d[0]));

    Logger.recordOutput(
        "Camera/Global/AprilTags/FieldPose2d",
        fieldPosesByTagId.values().toArray(new Pose2d[0]));
  }
}
