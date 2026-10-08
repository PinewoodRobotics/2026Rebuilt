package frc.robot.subsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.BotConstants;
import frc.robot.constant.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private static VisionSubsystem self;

  private final OdometrySubsystem odometry;
  private final List<VisionCamera> cameras = new ArrayList<>();

  private record VisionCamera(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
  }

  public static VisionSubsystem GetInstance() {
    if (self == null) {
      self = new VisionSubsystem(OdometrySubsystem.GetInstance());
    }
    return self;
  }

  public VisionSubsystem(OdometrySubsystem odometry) {
    this.odometry = odometry;
    for (VisionConstants.CameraConfig config : VisionConstants.kCameras) {
      if (config.enabled()) {
        cameras.add(new VisionCamera(
            new PhotonCamera(config.name()),
            new PhotonPoseEstimator(BotConstants.kFieldLayout, config.robotToCamera())));
      }
    }
  }

  @Override
  public void periodic() {
    for (VisionCamera cam : cameras) {
      String prefix = "Vision/" + cam.camera().getName();
      List<Pose2d> accepted = new ArrayList<>();
      List<Pose2d> rejected = new ArrayList<>();

      for (PhotonPipelineResult result : cam.camera().getAllUnreadResults()) {
        Optional<EstimatedRobotPose> estimate = estimatePose(cam.poseEstimator(), result);
        if (estimate.isEmpty()) {
          continue;
        }

        EstimatedRobotPose e = estimate.get();
        Pose2d pose = e.estimatedPose.toPose2d();
        if (!isOnField(e.estimatedPose)) {
          rejected.add(pose);
          continue;
        }

        boolean isMultiTag = e.targetsUsed.size() > 1;
        if (!odometry.isAnchoredToField() && isMultiTag) {
          odometry.resetPose(pose);
        } else {
          odometry.addVisionMeasurement(pose, e.timestampSeconds, stdDevsFor(e));
        }
        accepted.add(pose);
      }

      Logger.recordOutput(prefix + "/Connected", cam.camera().isConnected());
      Logger.recordOutput(prefix + "/AcceptedPoses", accepted.toArray(Pose2d[]::new));
      Logger.recordOutput(prefix + "/RejectedPoses", rejected.toArray(Pose2d[]::new));
    }
  }

  private static Optional<EstimatedRobotPose> estimatePose(PhotonPoseEstimator poseEstimator,
      PhotonPipelineResult result) {
    if (!result.hasTargets()) {
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(result);
    if (estimate.isPresent()) {
      return estimate;
    }

    var best = result.getBestTarget();
    if (best.getPoseAmbiguity() > VisionConstants.kMaxSingleTagAmbiguity
        || best.getBestCameraToTarget().getTranslation().getNorm() > VisionConstants.kMaxSingleTagDistanceM) {
      return Optional.empty();
    }
    return poseEstimator.estimateLowestAmbiguityPose(result);
  }

  private static boolean isOnField(Pose3d pose) {
    return pose.getX() >= 0 && pose.getX() <= BotConstants.kFieldLayout.getFieldLength()
        && pose.getY() >= 0 && pose.getY() <= BotConstants.kFieldLayout.getFieldWidth()
        && Math.abs(pose.getZ()) <= VisionConstants.kMaxPoseHeightErrorM;
  }

  private static Matrix<N3, N1> stdDevsFor(EstimatedRobotPose estimate) {
    double avgDistM = estimate.targetsUsed.stream()
        .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
        .average()
        .orElse(VisionConstants.kMaxSingleTagDistanceM);

    var base = estimate.targetsUsed.size() > 1 ? VisionConstants.kMultiTagStdDevs : VisionConstants.kSingleTagStdDevs;
    return base.times(1 + (avgDistM * avgDistM / 30));
  }
}
