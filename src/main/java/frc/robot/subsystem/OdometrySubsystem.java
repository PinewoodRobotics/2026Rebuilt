package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.TopicConstants;
import frc.robot.hardware.AHRSGyro;
import proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import proto.sensor.Odometry.OdometryData;
import proto.util.Position.Position2d;
import proto.util.Vector.Vector2;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;
import pwrup.frc.core.proto.IDataClass;

public class OdometrySubsystem extends SubsystemBase implements IDataClass {

  private static OdometrySubsystem self;
  private final SwerveSubsystem swerve;
  private final SwerveDriveOdometry odometry;
  private final IGyroscopeLike gyro;
  public Pose2d[] timedPositions = new Pose2d[2];
  public long[] timestamps = new long[2];

  public static OdometrySubsystem GetInstance() {
    if (self == null) {
      self = new OdometrySubsystem();
    }

    return self;
  }

  public OdometrySubsystem() {
    this.swerve = SwerveSubsystem.GetInstance();
    this.gyro = AHRSGyro.GetInstance();

    this.odometry = new SwerveDriveOdometry(
        swerve.getKinematics(),
        getGlobalGyroRotation(),
        swerve.getSwerveModulePositions(),
        new Pose2d(5, 5, new Rotation2d()));
  }

  public void setOdometryPosition(Pose2d newPose) {
    odometry.resetPosition(
        getGlobalGyroRotation(),
        swerve.getSwerveModulePositions(),
        newPose);
  }

  public Rotation2d getGlobalGyroRotation() {
    return Rotation2d.fromDegrees(-this.gyro.getYaw());
  }

  @Override
  public void periodic() {
    timedPositions[0] = timedPositions[1];
    timestamps[0] = timestamps[1];
    timestamps[1] = System.currentTimeMillis();

    var positions = swerve.getSwerveModulePositions();
    timedPositions[1] = odometry.update(getGlobalGyroRotation(), positions);
    Logger.recordOutput("odometry/pos", timedPositions[1]);
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    var all = GeneralSensorData.newBuilder().setOdometry(OdometryData.newBuilder());
    all.setSensorId("odom");

    var positionChange = timedPositions[1].minus(timedPositions[0]);
    var timeChange = ((timestamps[1] - timestamps[0]) + (System.currentTimeMillis() - timestamps[1])) / 1000.0;

    var latestPosition = timedPositions[1];

    var rotation = Vector2.newBuilder().setX((float) latestPosition.getRotation().getCos())
        .setY((float) latestPosition.getRotation().getSin())
        .build();
    var pose = Position2d.newBuilder()
        .setPosition(
            Vector2.newBuilder().setX((float) latestPosition.getX()).setY((float) latestPosition.getY()).build())
        .setDirection(rotation)
        .build();

    var velocity = Vector2.newBuilder().setX((float) SwerveSubsystem.GetInstance().getChassisSpeeds().vxMetersPerSecond)
        .setY((float) SwerveSubsystem.GetInstance().getChassisSpeeds().vyMetersPerSecond)
        .build();

    var positionChangeVec = Vector2.newBuilder().setX((float) positionChange.getX()).setY((float) positionChange.getY())
        .build();

    all.setOdometry(
        OdometryData.newBuilder().setPosition(pose).setVelocity(velocity).setPositionChange(positionChangeVec)
            .setTimeChangeS((float) timeChange).build());

    return all.build().toByteArray();
  }

  @Override
  public String getPublishTopic() {
    return TopicConstants.kOdometryPublishTopic;
  }
}
