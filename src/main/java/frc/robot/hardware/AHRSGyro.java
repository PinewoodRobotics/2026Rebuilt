package frc.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
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
  public double[] getYPR() {
    return new double[] {
        getYawDegrees(),
        m_gyro.getPitch(),
        m_gyro.getRoll()
    };
  }

  @Override
  public double[] getLinearAccelerationXYZ() {
    return new double[] {
        m_gyro.getWorldLinearAccelX(),
        m_gyro.getWorldLinearAccelY(),
        m_gyro.getWorldLinearAccelZ()
    };
  }

  @Override
  public double[] getAngularVelocityXYZ() {
    return new double[] {
        Math.toRadians(m_gyro.getRawGyroX()),
        Math.toRadians(m_gyro.getRawGyroY()),
        Math.toRadians(m_gyro.getRawGyroZ())
    };
  }

  @Override
  public double[] getQuaternion() {
    return new double[] {
        m_gyro.getQuaternionW(),
        m_gyro.getQuaternionX(),
        m_gyro.getQuaternionY(),
        m_gyro.getQuaternionZ()
    };
  }

  @Override
  public double[] getLinearVelocityXYZ() {
    return new double[] {
        m_gyro.getVelocityX(),
        m_gyro.getVelocityY(),
        m_gyro.getVelocityZ()
    };
  }

  @Override
  public double[] getPoseXYZ() {
    return new double[] {
        xOffset + m_gyro.getDisplacementX(),
        yOffset + m_gyro.getDisplacementY(),
        zOffset + m_gyro.getDisplacementZ()
    };
  }

  @Override
  public void reset() {
    m_gyro.reset();
    m_gyro.resetDisplacement();
    xOffset = 0;
    yOffset = 0;
    zOffset = 0;
    yawSoftOffsetDeg = 0.0;
    resetYawRateState();
  }

  @Override
  public void setAngleAdjustment(double angle) {
    yawSoftOffsetDeg = LocalMath.wrapTo180(angle);
    resetYawRateState();
  }

  @Override
  public void setPositionAdjustment(double x, double y, double z) {
    xOffset = x;
    yOffset = y;
    zOffset = z;
  }

  public void resetRotation(Rotation2d newRotation) {
    setYawDegrees(newRotation.getDegrees());
  }

  public void resetRotation(edu.wpi.first.math.geometry.Rotation3d newRotation) {
    resetRotation(newRotation.toRotation2d());
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    return null;
  }

  @Override
  public String getPublishTopic() {
    return "imu/imu";
  }

}
