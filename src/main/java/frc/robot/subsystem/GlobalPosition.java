package frc.robot.subsystem;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.google.protobuf.InvalidProtocolBufferException;

import autobahn.client.NamedCallback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constant.BotConstants;
import frc.robot.constant.CommunicationConstants;
import frc.robot.util.AimPoint;
import frc.robot.util.PositionUncertaintyVisualizer;
import frc.robot.util.PositionUncertaintyVisualizer.PositionVisualizationMode;
import frc4765.proto.util.Position.RobotPosition;
import lombok.Getter;

public class GlobalPosition extends SubsystemBase {
  private static volatile long lastUpdateTime;
  private static volatile double positionUpdateHz;
  private static GlobalPosition self;
  private static Pose2d position = new Pose2d(12.94, 3.52, new Rotation2d(1, 0));
  private static ChassisSpeeds positionVelocity = new ChassisSpeeds(0, 0, 0);
  private static double[] positionCovariance = new double[0];
  private static double[][] positionCovarianceMatrix = new double[0][0];

  private int resubCountTicks = 0;

  @Getter
  private static boolean isValid = false;
  private static final long kPositionUpdateTimeoutMs = 1000;

  public static enum GMFrame {
    kFieldRelative,
    kRobotRelative,
  }

  public static GlobalPosition GetInstance() {
    if (self == null) {
      self = new GlobalPosition();
    }
    return self;
  }

  public GlobalPosition() {
    lastUpdateTime = System.currentTimeMillis();
    Robot.getCommunicationClient().subscribe(CommunicationConstants.kPoseSubscribeTopic,
        NamedCallback.FromConsumer(this::subscription));
  }

  public void subscription(byte[] payload) {
    try {
      RobotPosition position = RobotPosition.parseFrom(payload);
      var pose = position.getPosition2D().getPosition();
      var velocity = position.getPosition2D().getVelocity();
      var direction = position.getPosition2D().getDirection();
      var rotationSpeed = position.getPosition2D().getRotationSpeedRadS();

      if (pose == null || (direction.getX() == 0 && direction.getY() == 0) || Double.isNaN(direction.getX())
          || Double.isNaN(direction.getY())) {
        System.out.println("Invalid position or direction! + " + pose + " + " + direction);
        return;
      }

      GlobalPosition.position = new Pose2d(pose.getX(),
          pose.getY(),
          new Rotation2d(direction.getX(), direction.getY()));

      positionVelocity = new ChassisSpeeds(velocity.getX(), velocity.getY(),
          rotationSpeed);
      positionCovariance = toDoubleArray(position.getPList());
      positionCovarianceMatrix = PositionUncertaintyVisualizer.covarianceMatrix(positionCovariance);

      long now = System.currentTimeMillis();
      positionUpdateHz = 1000.0 / ((double) (now - lastUpdateTime));
      lastUpdateTime = now;
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
      return;
    }
  }

  public static Pose2d Get() {
    return position;
  }

  public static long getLastUpdateTimeMs() {
    return lastUpdateTime;
  }

  public static double[] getPositionCovariance() {
    return Arrays.copyOf(positionCovariance, positionCovariance.length);
  }

  public static double[][] getPositionCovarianceMatrix() {
    double[][] copy = new double[positionCovarianceMatrix.length][];
    for (int i = 0; i < positionCovarianceMatrix.length; i++) {
      copy[i] = Arrays.copyOf(positionCovarianceMatrix[i], positionCovarianceMatrix[i].length);
    }
    return copy;
  }

  public static Translation2d Velocity2d(GMFrame velocityType) {
    var velocity = Velocity(velocityType);
    return new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
  }

  public static ChassisSpeeds Velocity(GMFrame velocityType) {
    if (velocityType == GMFrame.kFieldRelative) {
      return positionVelocity;
    } else if (velocityType == GMFrame.kRobotRelative) {
      return VelocityInFrame(position.getRotation());
    }

    return positionVelocity;
  }

  /**
   * Converts the velocity from the field frame to the robot frame.
   * 
   * @param rotationOfRobot The rotation of the robot in the field frame. This is
   *                        the angle of the robot in the field frame.
   * @return The velocity in the robot frame.
   */
  public static ChassisSpeeds VelocityInFrame(Rotation2d rotationOfRobot) {
    if (positionVelocity == null) {
      return null;
    }

    return ChassisSpeeds.fromFieldRelativeSpeeds(positionVelocity, rotationOfRobot);
  }

  public static Pose2d ToRobotRelative(Pose2d pose) {
    return pose.relativeTo(position);
  }

  public static Translation2d ToRobotRelative(Translation2d translation) {
    return translation.rotateBy(position.getRotation());
  }

  @Override
  public void periodic() {
    Pose2d[] uncertaintyVisualization = PositionUncertaintyVisualizer.visualization(
        position,
        positionCovariance,
        PositionVisualizationMode.PROBABILITY_ELLIPSE);

    Pose2d[] covarianceVisualization = PositionUncertaintyVisualizer.visualization(
        position,
        positionCovariance,
        PositionVisualizationMode.COVARIANCE_ELLIPSE);

    Logger.recordOutput("Global/pose", position);
    Logger.recordOutput("Global/velocity", positionVelocity);
    Logger.recordOutput("Global/positionUncertaintyVisualization", uncertaintyVisualization);
    Logger.recordOutput("Global/covarianceVisualization", covarianceVisualization);
    if (positionUpdateHz < 100) {
      Logger.recordOutput("Global/positionUpdateHz", positionUpdateHz);
    }

    for (AimPoint.ZoneName zoneName : AimPoint.ZoneName.values()) {
      AimPoint.logZoneForAdvantageScope(zoneName, "Global/Zones/All");
    }

    if (position != null) {
      AimPoint.ZoneName activeZone = AimPoint.getZone(position);
      AimPoint.logZoneForAdvantageScope(activeZone, "Global/Zones/Active");

      var target = AimPoint.getTarget(activeZone);
      Logger.recordOutput(
          "Global/Zones/Active/LineToTarget",
          new Pose2d[] {
              new Pose2d(position.getTranslation(), new Rotation2d()),
              new Pose2d(target, new Rotation2d())
          });
    }

    if (System.currentTimeMillis() - lastUpdateTime > kPositionUpdateTimeoutMs) {
      isValid = false;
    } else {
      isValid = true;
    }

    Logger.recordOutput("Global/Position/IsValid", isValid);
    Logger.recordOutput("Global/alliance", BotConstants.alliance);

    if (positionUpdateHz < 10) {
      resubCountTicks++;
      if (resubCountTicks % 50 == 0) {
        Robot.getCommunicationClient().unsubscribe(CommunicationConstants.kPoseSubscribeTopic);
        Robot.getCommunicationClient().subscribe(CommunicationConstants.kPoseSubscribeTopic,
            NamedCallback.FromConsumer(this::subscription));

        System.out.println("Resubscribing to topic!!");

        resubCountTicks = 0;
      }
    } else {
      resubCountTicks = 0;
    }
  }

  private static double[] toDoubleArray(java.util.List<Float> values) {
    double[] result = new double[values.size()];
    for (int i = 0; i < values.size(); i++) {
      result[i] = values.get(i);
    }
    return result;
  }
}
