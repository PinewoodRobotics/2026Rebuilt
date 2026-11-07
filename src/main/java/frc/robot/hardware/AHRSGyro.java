package frc.robot.hardware;

import com.studica.frc.Navx;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.robot.util.LocalMath;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;

public class AHRSGyro implements IGyroscopeLike {
  private static AHRSGyro instance;
  private final Navx m_gyro;
  private double xOffset = 0;
  private double yOffset = 0;
  private double zOffset = 0;
  private double yawSoftOffsetDeg = 0.0;
  private boolean hasYawRateSample = false;
  private double lastYawRateSampleDeg = 0.0;
  private long lastYawRateSampleNanos = 0L;

  public AHRSGyro(Navx navx) {
    this.m_gyro = navx;
    m_gyro.resetYaw();
    yawSoftOffsetDeg = 0.0;
    resetYawRateState();
  }

  public static AHRSGyro GetInstance() {
    if (instance == null) {
      instance = new AHRSGyro(Navx.usb());
    }
    return instance;
  }

  public Navx getGyro() {
    return m_gyro;
  }

  private void resetYawRateState() {
    hasYawRateSample = false;
    lastYawRateSampleDeg = 0.0;
    lastYawRateSampleNanos = 0L;
  }

  private double getRawYawDegrees() {
    return m_gyro.getRotation2d().map(Rotation2d::getDegrees).orElse(0.0);
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
  public ChassisVelocities getVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
  }

  @Override
  public ChassisVelocities getAcceleration() {
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
