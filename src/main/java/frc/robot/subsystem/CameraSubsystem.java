package frc.robot.subsystem;

import java.io.IOException;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constant.PiConstants;
import proto.sensor.Apriltags.ProcessedTag;
import proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import proto.sensor.GeneralSensorDataOuterClass.SensorName;

public class CameraSubsystem extends SubsystemBase {
  private static CameraSubsystem instance;

  private static final String FIELD_LAYOUT_DEPLOY_FILE = "2026-rebuilt-welded.json";
  private static final AprilTagFieldLayout FIELD_LAYOUT = loadFieldLayout();
  private static final int POSE_LOG_DIVISOR = 5; // Log pose arrays at ~10Hz (50Hz / 5)

  private static AprilTagFieldLayout loadFieldLayout() {
    try {
      var path = Filesystem.getDeployDirectory().toPath().resolve(FIELD_LAYOUT_DEPLOY_FILE);
      return new AprilTagFieldLayout(path);
    } catch (IOException e) {
      DriverStation.reportError(
          "Failed to load AprilTag field layout from deploy file '" + FIELD_LAYOUT_DEPLOY_FILE
              + "'. AprilTag field poses will be unavailable.",
          e.getStackTrace());
      // Create an empty layout so we don't crash robot init.
      return new AprilTagFieldLayout(List.of(), 0.0, 0.0);
    }
  }

  private static class CameraMetrics {
    volatile GeneralSensorData lastSensorData;
    volatile double lastProcessingTimeMs;
    volatile double lastFps;
    volatile long lastArrivalTimeMs;

    // Derived/cached values for logging (computed on message arrival, not in
    // periodic()).
    volatile Pose2d[] lastCameraRelativePoses2d = new Pose2d[0];
    volatile int[] lastFieldTagIds = new int[0];
    volatile Pose2d[] lastFieldPoses2d = new Pose2d[0];
  }

  // Metrics and last packets for each camera (keyed by sensor_id from
  // GeneralSensorData).
  private final Map<String, CameraMetrics> metricsBySensor = new ConcurrentHashMap<>();
  private int poseLogCounter = 0;

  public static CameraSubsystem GetInstance() {
    if (instance == null) {
      instance = new CameraSubsystem();
    }
    return instance;
  }

  private CameraSubsystem() {
    Robot.getCommunicationClient().subscribe(
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

      // Precompute per-tag poses here so periodic() doesn't allocate or loop tags.
      var apriltags = data.getApriltags();
      var worldTags = apriltags.getWorldTags();
      List<ProcessedTag> tags = worldTags.getTagsList();

      Pose2d[] cameraRelativePoses = new Pose2d[tags.size()];
      int[] fieldIdsTmp = new int[tags.size()];
      Pose2d[] fieldPosesTmp = new Pose2d[tags.size()];
      int fieldCount = 0;

      for (int i = 0; i < tags.size(); i++) {
        ProcessedTag tag = tags.get(i);
        int id = tag.getId();

        var pos = tag.getPositionWPILib();
        var rot = tag.getRotationWPILib();

        cameraRelativePoses[i] = new Pose2d(
            new Translation2d(pos.getX(), pos.getY()),
            new Rotation2d(rot.getYaw()));

        var fieldPoseOpt = FIELD_LAYOUT.getTagPose(id);
        if (fieldPoseOpt.isPresent()) {
          fieldIdsTmp[fieldCount] = id;
          fieldPosesTmp[fieldCount] = fieldPoseOpt.get().toPose2d();
          fieldCount++;
        }
      }

      if (fieldCount != fieldIdsTmp.length) {
        int[] fieldIds = new int[fieldCount];
        Pose2d[] fieldPoses = new Pose2d[fieldCount];
        System.arraycopy(fieldIdsTmp, 0, fieldIds, 0, fieldCount);
        System.arraycopy(fieldPosesTmp, 0, fieldPoses, 0, fieldCount);
        metrics.lastFieldTagIds = fieldIds;
        metrics.lastFieldPoses2d = fieldPoses;
      } else {
        metrics.lastFieldTagIds = fieldIdsTmp;
        metrics.lastFieldPoses2d = fieldPosesTmp;
      }

      metrics.lastCameraRelativePoses2d = cameraRelativePoses;
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
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

  /*
   * @Override
   * public void periodic() {
   * double globalFps = 0.0;
   * 
   * long now = System.currentTimeMillis();
   * 
   * for (Map.Entry<String, CameraMetrics> entry : metricsBySensor.entrySet()) {
   * String sensorId = entry.getKey();
   * CameraMetrics metrics = entry.getValue();
   * String prefix = "Camera/" + sensorId + "/AprilTags/";
   * Logger.recordOutput(prefix + "frameProcessingMs",
   * metrics.lastProcessingTimeMs);
   * Logger.recordOutput(prefix + "fps", metrics.lastFps);
   * 
   * globalFps += metrics.lastFps;
   * 
   * long ageMs = metrics.lastArrivalTimeMs > 0 ? now - metrics.lastArrivalTimeMs
   * : 0;
   * double estimatedLatencyMs = ageMs + metrics.lastProcessingTimeMs;
   * 
   * // Age of the last frame seen from this camera and an approximate end-to-end
   * // latency (capture -> backend processing -> transport -> robot use),
   * // computed without trusting any remote timestamps.
   * Logger.recordOutput(prefix + "frameAgeMs", ageMs);
   * Logger.recordOutput(prefix + "estimatedLatencyMs", estimatedLatencyMs);
   * }
   * 
   * Logger.recordOutput("Camera/Global/AprilTags/fpsCombined", globalFps);
   * 
   * boolean shouldLogPoses = (poseLogCounter++ % POSE_LOG_DIVISOR) == 0;
   * if (shouldLogPoses) {
   * List<Pose2d> cameraRelativePoses = new ArrayList<>();
   * Map<Integer, Pose2d> fieldPosesByTagId = new LinkedHashMap<>();
   * 
   * for (CameraMetrics metrics : metricsBySensor.values()) {
   * Pose2d[] cameraPoses = metrics.lastCameraRelativePoses2d;
   * if (cameraPoses != null && cameraPoses.length > 0) {
   * for (Pose2d p : cameraPoses) {
   * cameraRelativePoses.add(p);
   * }
   * }
   * 
   * int[] fieldIds = metrics.lastFieldTagIds;
   * Pose2d[] fieldPoses = metrics.lastFieldPoses2d;
   * if (fieldIds != null && fieldPoses != null) {
   * int n = Math.min(fieldIds.length, fieldPoses.length);
   * for (int i = 0; i < n; i++) {
   * fieldPosesByTagId.put(fieldIds[i], fieldPoses[i]);
   * }
   * }
   * }
   * 
   * Logger.recordOutput(
   * "Camera/Global/AprilTags/CameraRelativePose2d",
   * cameraRelativePoses.toArray(new Pose2d[cameraRelativePoses.size()]));
   * 
   * Logger.recordOutput(
   * "Camera/Global/AprilTags/FieldPose2d",
   * fieldPosesByTagId.values().toArray(new Pose2d[fieldPosesByTagId.size()]));
   * }
   * }
   */
}
