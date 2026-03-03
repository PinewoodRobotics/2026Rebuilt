package frc.robot.hardware;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.BotConstants;
import frc.robot.constant.HardwareConstants;
import frc4765.proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import frc4765.proto.sensor.GeneralSensorDataOuterClass.SensorName;
import frc4765.proto.sensor.Imu.ImuData;
import frc4765.proto.util.Position.Position3d;
import frc4765.proto.util.Vector.Vector3;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;
import pwrup.frc.core.proto.IDataClass;

public class PigeonGyro extends SubsystemBase implements IGyroscopeLike, IDataClass {
  private static PigeonGyro instance;

  private final Pigeon2 pigeon;
  private Rotation2d yawAdjustment = new Rotation2d();

  public PigeonGyro(int canId) {
    this.pigeon = new Pigeon2(canId);
    applyMountPose();
    pigeon.reset();
    yawAdjustment = new Rotation2d();
  }

  public static PigeonGyro GetInstance() {
    if (instance == null) {
      instance = new PigeonGyro(HardwareConstants.kPigeonCanId);
    }

    return instance;
  }

  @Override
  public ChassisSpeeds getVelocity() {
    return new ChassisSpeeds(0.0, 0.0, pigeon.getAngularVelocityZWorld().getValue().in(Units.RadiansPerSecond));
  }

  @Override
  public ChassisSpeeds getAcceleration() {
    return new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  @Override
  public Rotation3d getRotation() {
    return new Rotation3d(0.0, 0.0, getYawRotation2d().getRadians());
  }

  @Override
  public void resetRotation(Rotation3d newRotation) {
    yawAdjustment = newRotation.toRotation2d().minus(getRawYawRotation2d());
  }

  private Rotation2d getRawYawRotation2d() {
    return pigeon.getRotation2d();
  }

  private Rotation2d getYawRotation2d() {
    return getRawYawRotation2d().plus(yawAdjustment);
  }

  private void applyMountPose() {
    var config = new Pigeon2Configuration();
    config.MountPose.withMountPoseYaw(HardwareConstants.kPigeonMountPoseYawDeg);
    config.MountPose.withMountPosePitch(HardwareConstants.kPigeonMountPosePitchDeg);
    config.MountPose.withMountPoseRoll(HardwareConstants.kPigeonMountPoseRollDeg);

    var status = pigeon.getConfigurator().apply(config);
    if (!status.isOK()) {
      DriverStation.reportWarning("Failed to apply Pigeon mount pose: " + status, false);
    }
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    Rotation2d rotation = getRotation().toRotation2d();
    ChassisSpeeds velocity = getVelocity();
    ChassisSpeeds acceleration = getAcceleration();

    var position = Vector3.newBuilder()
        .setX(0.0f)
        .setY(0.0f)
        .setZ(0.0f)
        .build();

    var direction = Vector3.newBuilder()
        .setX((float) rotation.getCos())
        .setY((float) rotation.getSin())
        .setZ(0)
        .build();

    var position2d = Position3d.newBuilder()
        .setPosition(position)
        .setDirection(direction)
        .build();

    var vel = Vector3.newBuilder()
        .setX((float) velocity.vxMetersPerSecond)
        .setY((float) velocity.vyMetersPerSecond)
        .setZ(0.0f)
        .build();

    var acc = Vector3.newBuilder()
        .setX((float) acceleration.vxMetersPerSecond)
        .setY((float) acceleration.vyMetersPerSecond)
        .setZ(0.0f)
        .build();

    var angularVel = Vector3.newBuilder().setX((float) 0.0).setY((float) 0.0)
        .setZ((float) velocity.omegaRadiansPerSecond)
        .build();

    var imuData = ImuData.newBuilder()
        .setPosition(position2d)
        .setVelocity(vel)
        .setAcceleration(acc)
        .setAngularVelocityXYZ(angularVel)
        .build();

    var all = GeneralSensorData.newBuilder().setImu(imuData).setSensorName(SensorName.IMU).setSensorId("1")
        .setTimestamp(System.currentTimeMillis()).setProcessingTimeMs(0);

    return all.build().toByteArray();
  }

  @Override
  public String getPublishTopic() {
    return "imu/imu";
  }

  @Override
  public void periodic() {
    Logger.recordOutput("PigeonGyro/velocity", getVelocity());
    Logger.recordOutput("PigeonGyro/acceleration", getAcceleration());
    Logger.recordOutput("PigeonGyro/Rotation/rotation", getRotation());
    Logger.recordOutput("PigeonGyro/Rotation/rotation2d", getRotation().toRotation2d());
    Logger.recordOutput("PigeonGyro/Rotation/cos", getRotation().toRotation2d().getCos());
    Logger.recordOutput("PigeonGyro/Rotation/sin", getRotation().toRotation2d().getSin());
  }
}
