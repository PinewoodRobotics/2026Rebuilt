package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.CommunicationConstants;
import frc.robot.hardware.UnifiedGyro;
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
  public Pose2d[] timedPositions = new Pose2d[] { new Pose2d(), new Pose2d() };
  private final SwerveModulePosition[][] timedModulePositions = new SwerveModulePosition[2][];
  public long[] timestamps = new long[2];
  private SwerveModulePosition[] lastPublishedModulePositions;
  private long lastPublishedTimestampMs;

  public static OdometrySubsystem GetInstance(IGyroscopeLike gyro, SwerveSubsystem swerve) {
    if (self == null) {
      self = new OdometrySubsystem(gyro, swerve);
    }

    return self;
  }

  public static OdometrySubsystem GetInstance(IGyroscopeLike gyro) {
    return GetInstance(gyro, SwerveSubsystem.GetInstance());
  }

  public static OdometrySubsystem GetInstance() {
    return GetInstance(UnifiedGyro.GetInstance(), SwerveSubsystem.GetInstance());
  }

  public OdometrySubsystem(IGyroscopeLike gyro, SwerveSubsystem swerve) {
    this.gyro = gyro;
    this.swerve = swerve;
    SwerveModulePosition[] initialModulePositions = copyModulePositions(swerve.getSwerveModulePositions());
    Pose2d initialPose = new Pose2d(5, 5, new Rotation2d());
    this.odometry = new SwerveDriveOdometry(
        swerve.getKinematics(),
        gyro.getRotation2d(),
        initialModulePositions,
        initialPose);
    timedPositions[0] = initialPose;
    timedPositions[1] = initialPose;
    timedModulePositions[0] = copyModulePositions(initialModulePositions);
    timedModulePositions[1] = copyModulePositions(initialModulePositions);
    timestamps[0] = System.currentTimeMillis();
    timestamps[1] = timestamps[0];
    lastPublishedModulePositions = copyModulePositions(initialModulePositions);
    lastPublishedTimestampMs = timestamps[1];
  }

  public void setOdometryPosition(Pose2d newPose) {
    SwerveModulePosition[] currentModulePositions = copyModulePositions(swerve.getSwerveModulePositions());
    odometry.resetPosition(
        gyro.getRotation2d(),
        currentModulePositions,
        newPose);
    timedPositions[0] = newPose;
    timedPositions[1] = newPose;
    timedModulePositions[0] = copyModulePositions(currentModulePositions);
    timedModulePositions[1] = copyModulePositions(currentModulePositions);
    timestamps[0] = System.currentTimeMillis();
    timestamps[1] = timestamps[0];
    lastPublishedModulePositions = copyModulePositions(currentModulePositions);
    lastPublishedTimestampMs = timestamps[1];
  }

  private Pose2d getLatestPosition() {
    return timedPositions[1];
  }

  private Transform2d getPoseDifferenceSinceLastPublish() {
    Twist2d wheelDelta = swerve.getKinematics().toTwist2d(
        lastPublishedModulePositions,
        timedModulePositions[1]);
    return new Transform2d(
        wheelDelta.dx,
        wheelDelta.dy,
        new Rotation2d(wheelDelta.dtheta));
  }

  private static SwerveModulePosition[] copyModulePositions(SwerveModulePosition[] positions) {
    SwerveModulePosition[] copy = new SwerveModulePosition[positions.length];
    for (int i = 0; i < positions.length; i++) {
      copy[i] = new SwerveModulePosition(
          positions[i].distanceMeters,
          positions[i].angle);
    }
    return copy;
  }

  private double getTimeDifferenceSinceLastPublish() {
    return Math.max(0.0, (timestamps[1] - lastPublishedTimestampMs) / 1000.0);
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    var all = GeneralSensorData.newBuilder().setOdometry(OdometryData.newBuilder());
    all.setSensorId(CommunicationConstants.kOdometrySensorId);

    var positionChange = getPoseDifferenceSinceLastPublish();
    var timeChange = getTimeDifferenceSinceLastPublish();
    var latestPosition = getLatestPosition();
    var chassisSpeeds = SwerveSubsystem.GetInstance().getChassisSpeeds();

    var rotation = Vector2.newBuilder().setX((float) latestPosition.getRotation().getCos())
        .setY((float) latestPosition.getRotation().getSin())
        .build();

    var pose = Position2d.newBuilder()
        .setPosition(
            Vector2.newBuilder().setX((float) latestPosition.getX()).setY((float) latestPosition.getY()).build())
        .setDirection(rotation)
        .build();

    var velocity = Vector2.newBuilder().setX((float) chassisSpeeds.vxMetersPerSecond)
        .setY((float) chassisSpeeds.vyMetersPerSecond)
        .build();

    var positionChangeVec = Vector2.newBuilder().setX((float) positionChange.getX()).setY((float) positionChange.getY())
        .build();

    all.setOdometry(
        OdometryData.newBuilder()
            .setPosition(pose)
            .setVelocity(velocity)
            .setPositionChange(positionChangeVec)
            .setTimeChangeS((float) timeChange)
            .setOmega((float) chassisSpeeds.omegaRadiansPerSecond)
            .build())
        .setTimestamp(System.currentTimeMillis());

    lastPublishedModulePositions = copyModulePositions(timedModulePositions[1]);
    lastPublishedTimestampMs = timestamps[1];

    return all.build().toByteArray();
  }

  @Override
  public String getPublishTopic() {
    return CommunicationConstants.kOdometryPublishTopic;
  }

  @Override
  public void periodic() {
    timedPositions[0] = timedPositions[1];
    timedModulePositions[0] = timedModulePositions[1] == null ? null : copyModulePositions(timedModulePositions[1]);
    timestamps[0] = timestamps[1];
    timestamps[1] = System.currentTimeMillis();

    var positions = swerve.getSwerveModulePositions();
    timedModulePositions[1] = copyModulePositions(positions);
    timedPositions[1] = odometry.update(gyro.getRotation2d(), positions);

    Logger.recordOutput("Odometry/Position", timedPositions[1]);
    Logger.recordOutput("Odometry/Velocity", swerve.getChassisSpeeds());
  }
}
