package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.CommunicationConstants;
import frc.robot.hardware.PigeonGyro;
import frc4765.proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import frc4765.proto.sensor.Odometry.OdometryData;
import frc4765.proto.util.Position.Position2d;
import frc4765.proto.util.Vector.Vector2;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;
import pwrup.frc.core.proto.IDataClass;

public class OdometrySubsystem extends SubsystemBase implements IDataClass {

  private static OdometrySubsystem self;
  private final SwerveSubsystem swerve;
  private final SwerveDriveOdometry odometry;
  private final IGyroscopeLike gyro;
  public Pose2d[] timedPositions = new Pose2d[2];
  public long[] timestamps = new long[2];

  public static OdometrySubsystem GetInstance(IGyroscopeLike gyro, SwerveSubsystem swerve) {
    if (self == null) {
      self = new OdometrySubsystem(gyro, swerve);
    }

    return self;
  }

  public static OdometrySubsystem GetInstance() {
    return GetInstance(PigeonGyro.GetInstance(), SwerveSubsystem.GetInstance());
  }

  public OdometrySubsystem(IGyroscopeLike gyro, SwerveSubsystem swerve) {
    this.gyro = gyro;
    this.swerve = swerve;
    this.odometry = new SwerveDriveOdometry(
        swerve.getKinematics(),
        gyro.getRotation2d(),
        swerve.getSwerveModulePositions(),
        new Pose2d(5, 5, new Rotation2d()));
  }

  public void setOdometryPosition(Pose2d newPose) {
    odometry.resetPosition(
        gyro.getRotation2d(),
        swerve.getSwerveModulePositions(),
        newPose);
    timedPositions[0] = newPose;
    timedPositions[1] = newPose;
  }

  private Pose2d getLatestPosition() {
    return timedPositions[1];
  }

  private Transform2d getPoseDifference() {
    return timedPositions[1].minus(timedPositions[0]);
  }

  private double getTimeDifference() {
    return ((timestamps[1] - timestamps[0]) + (System.currentTimeMillis() - timestamps[1])) / 1000.0;
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    var all = GeneralSensorData.newBuilder().setOdometry(OdometryData.newBuilder());
    all.setSensorId(CommunicationConstants.kOdometrySensorId);

    var positionChange = getPoseDifference();
    var timeChange = getTimeDifference();
    var latestPosition = getLatestPosition();

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
        OdometryData.newBuilder()
            .setPosition(pose)
            .setVelocity(velocity)
            .setPositionChange(positionChangeVec)
            .setTimeChangeS((float) timeChange)
            .build());

    return all.build().toByteArray();
  }

  @Override
  public String getPublishTopic() {
    return CommunicationConstants.kOdometryPublishTopic;
  }

  @Override
  public void periodic() {
    timedPositions[0] = timedPositions[1];
    timestamps[0] = timestamps[1];
    timestamps[1] = System.currentTimeMillis();

    var positions = swerve.getSwerveModulePositions();
    timedPositions[1] = odometry.update(gyro.getRotation2d(), positions);

    Logger.recordOutput("Odometry/Position", timedPositions[1]);
  }
}
