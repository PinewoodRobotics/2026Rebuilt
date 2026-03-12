package frc.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.util.LocalMath;
import frc4765.proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import frc4765.proto.sensor.GeneralSensorDataOuterClass.SensorName;
import frc4765.proto.sensor.Imu.ImuData;
import frc4765.proto.util.Position.Position3d;
import frc4765.proto.util.Vector.Vector3;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;
import pwrup.frc.core.proto.IDataClass;

public class AHRSGyro implements IGyroscopeLike, IDataClass {
  private static AHRSGyro instance;
  private static I2C.Port defaultPort = I2C.Port.kMXP;

  private final AHRS m_gyro;
  private double xOffset = 0;
  private double yOffset = 0;
  private double zOffset = 0;
  private double yawSoftOffsetDeg = 0.0;
  private boolean hasYawRateSample = false;
  private double lastYawRateSampleDeg = 0.0;
  private long lastYawRateSampleNanos = 0L;

  public AHRSGyro(I2C.Port i2c_port_id) {
    this.m_gyro = new AHRS(i2c_port_id);
    m_gyro.reset();
    yawSoftOffsetDeg = 0.0;
    resetYawRateState();
  }

  /**
   * Set the default I2C port used by GetInstance().
   * Call this before the first call to GetInstance().
   */
  public static void setDefaultPort(I2C.Port port) {
    defaultPort = port;
  }

  public static AHRSGyro GetInstance() {
    if (instance == null) {
      instance = new AHRSGyro(defaultPort);
    }
    return instance;
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  private void resetYawRateState() {
    hasYawRateSample = false;
    lastYawRateSampleDeg = 0.0;
    lastYawRateSampleNanos = 0L;
  }

  private double getRawYawDegrees() {
    return m_gyro.getRotation2d().getDegrees();
  }

  private double getAdjustedYawDegrees() {
    return LocalMath.wrapTo180(getRawYawDegrees() + yawSoftOffsetDeg);
  }

  private double getYawRateRadPerSec() {
    final long nowNanos = System.nanoTime();
    final double nowYawDeg = getAdjustedYawDegrees();

    if (!hasYawRateSample) {
      hasYawRateSample = true;
      lastYawRateSampleDeg = nowYawDeg;
      lastYawRateSampleNanos = nowNanos;
      return 0.0;
    }

    final double dtS = (nowNanos - lastYawRateSampleNanos) * 1e-9;
    final double deltaYawDeg = LocalMath.wrapTo180(nowYawDeg - lastYawRateSampleDeg);

    lastYawRateSampleDeg = nowYawDeg;
    lastYawRateSampleNanos = nowNanos;

    if (dtS <= 1e-6) {
      return 0.0;
    }

    return Math.toRadians(deltaYawDeg / dtS);
  }

  public void setYawDegrees(double yawDeg) {
    yawSoftOffsetDeg = LocalMath.wrapTo180(yawDeg - getRawYawDegrees());
    resetYawRateState();
  }

  public double getYawDegrees() {
    return getAdjustedYawDegrees();
  }

  public double getYawRadians() {
    return Math.toRadians(getYawDegrees());
  }

  public void setYawDeg(double targetDeg) {
    setYawDegrees(targetDeg);
  }

  public Rotation2d getNoncontinuousAngle() {
    return Rotation2d.fromDegrees(getYawDegrees());
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    return null;
  }

  @Override
  public String getPublishTopic() {
    return "imu/imu";
  }

  @Override
  public ChassisSpeeds getVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
  }

  @Override
  public ChassisSpeeds getAcceleration() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAcceleration'");
  }

  @Override
  public Rotation3d getRotation() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRotation'");
  }

  @Override
  public void resetRotation(Rotation3d newRotation) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetRotation'");
  }
}
