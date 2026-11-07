package frc.robot.hardware;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.Units;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.command2.SubsystemBase;
import frc.robot.constant.BotConstants;
import frc.robot.constant.HardwareConstants;
import frc.robot.constant.HardwareConstants.PigeonConfig;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;

public class PigeonGyro extends SubsystemBase implements IGyroscopeLike {
  private static PigeonGyro instance;

  private final Pigeon2 pigeon;
  private Rotation2d yawAdjustment = new Rotation2d();
  private PigeonConfig hardwareConfig;

  public PigeonGyro(PigeonConfig config) {
    this.hardwareConfig = config;
    this.pigeon = new Pigeon2(config.canId(), BotConstants.kCanBus);
    applyMountPose();
    pigeon.reset();
    yawAdjustment = new Rotation2d();
  }

  public static PigeonGyro GetInstance(PigeonConfig config) {
    if (instance == null) {
      instance = new PigeonGyro(config);
    }
    return instance;
  }

  @Override
  public ChassisVelocities getVelocity() {
    return new ChassisVelocities(0.0, 0.0, pigeon.getAngularVelocityZWorld().getValue().in(Units.RadiansPerSecond));
  }

  @Override
  public ChassisVelocities getAcceleration() {
    return new ChassisVelocities(0.0, 0.0, 0.0);
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
    config.MountPose.withMountPoseYaw(hardwareConfig.mountPoseYawDeg());
    config.MountPose.withMountPosePitch(hardwareConfig.mountPosePitchDeg());
    config.MountPose.withMountPoseRoll(hardwareConfig.mountPoseRollDeg());

    var status = pigeon.getConfigurator().apply(config);
    if (!status.isOK()) {
      DriverStationErrors.reportWarning("Failed to apply Pigeon mount pose: " + status, false);
    }
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
