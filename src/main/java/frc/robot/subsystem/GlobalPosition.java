package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.google.protobuf.InvalidProtocolBufferException;

import autobahn.client.NamedCallback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constant.TopicConstants;
import frc4765.proto.util.Position.RobotPosition;

public class GlobalPosition extends SubsystemBase {
  private static volatile long lastUpdateTime;
  private static GlobalPosition self;
  private static Pose2d position;
  private static ChassisSpeeds positionVelocity;

  public static GlobalPosition GetInstance() {
    if (self == null) {
      self = new GlobalPosition();
    }
    return self;
  }

  public GlobalPosition() {
    Robot.getCommunicationClient().subscribe(TopicConstants.kPoseSubscribeTopic,
        NamedCallback.FromConsumer(this::subscription));
  }

  public void subscription(byte[] payload) {
    try {
      RobotPosition position = RobotPosition.parseFrom(payload);
      var pose = position.getPosition2D().getPosition();
      var velocity = position.getPosition2D().getVelocity();
      var direction = position.getPosition2D().getDirection();
      var rotationSpeed = position.getPosition2D().getRotationSpeedRadS();

      GlobalPosition.position = new Pose2d(pose.getX(),
          pose.getY(),
          new Rotation2d(direction.getX(), direction.getY()));

      positionVelocity = new ChassisSpeeds(velocity.getX(), velocity.getY(),
          rotationSpeed);

      lastUpdateTime = (long) System.currentTimeMillis();
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
      return;
    }
  }

  public static Pose2d Get() {
    return position;
  }

  public static ChassisSpeeds GetVelocity() {
    return positionVelocity;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Global/pose", position);
    Logger.recordOutput("Global/velocity", positionVelocity);
    Logger.recordOutput("Global/lastUpdateTime", lastUpdateTime);
  }
}
