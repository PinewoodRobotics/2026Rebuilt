package frc.robot.hardware;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.util.CustomMath;
import proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import proto.sensor.GeneralSensorDataOuterClass.SensorName;
import proto.sensor.Imu.ImuData;
import proto.util.Position.Position3d;
import proto.util.Vector.Vector3;
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

  public AHRSGyro(I2C.Port i2c_port_id) {
    this.m_gyro = new AHRS(i2c_port_id);
    m_gyro.reset();
    yawSoftOffsetDeg = 0.0;
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

  @Override
  public double[] getYPR() {
    double yawAdj = CustomMath.wrapTo180(m_gyro.getYaw() + yawSoftOffsetDeg);
    return new double[] {
        yawAdj,
        m_gyro.getPitch(),
        m_gyro.getRoll(),
    };
  }

  @Override
  public void setPositionAdjustment(double x, double y, double z) {
    xOffset = x;
    yOffset = y;
    zOffset = z;
    m_gyro.resetDisplacement();
  }

  @Override
  public double[] getLinearAccelerationXYZ() {
    return new double[] {
        m_gyro.getWorldLinearAccelX(),
        m_gyro.getWorldLinearAccelY(),
        m_gyro.getWorldLinearAccelZ(),
    };
  }

  @Override
  public double[] getAngularVelocityXYZ() {
    return new double[] { 0, 0, Math.toRadians(m_gyro.getRate()) };
  }

  @Override
  public double[] getQuaternion() {
    return new double[] {
        m_gyro.getQuaternionW(),
        m_gyro.getQuaternionX(),
        m_gyro.getQuaternionY(),
        m_gyro.getQuaternionZ(),
    };
  }

  @Override
  public double[] getLinearVelocityXYZ() {
    return new double[] {
        m_gyro.getVelocityX(),
        m_gyro.getVelocityY(),
        m_gyro.getVelocityZ(),
    };
  }

  @Override
  public double[] getPoseXYZ() {
    return new double[] {
        m_gyro.getDisplacementX() + xOffset,
        m_gyro.getDisplacementY() + yOffset,
        m_gyro.getDisplacementZ() + zOffset,
    };
  }

  @Override
  public void reset() {
    m_gyro.reset();
    yawSoftOffsetDeg = 0.0;
  }

  @Override
  public void setAngleAdjustment(double angle) {
    m_gyro.zeroYaw();
    yawSoftOffsetDeg = -angle;
  }

  public void setYawDeg(double targetDeg) {
    setAngleAdjustment(targetDeg);
  }

  public Rotation2d getNoncontinuousAngle() {
    return Rotation2d.fromDegrees(CustomMath.wrapTo180(m_gyro.getAngle()));
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    var poseXYZ = getPoseXYZ();
    var velocityXYZ = getLinearVelocityXYZ();
    var accelerationXYZ = getLinearAccelerationXYZ();
    var yaw = Rotation2d.fromDegrees(getYPR()[0]);
    var angularVelocity = getAngularVelocityXYZ();

    Logger.recordOutput("Imu/AngularVel", angularVelocity[2]);

    Logger.recordOutput("Imu/yaw", yaw.getDegrees());

    var position = Vector3.newBuilder()
        .setX((float) poseXYZ[0])
        .setY((float) poseXYZ[1])
        .setZ((float) poseXYZ[2])
        .build();

    var direction = Vector3.newBuilder()
        .setX((float) yaw.getCos())
        .setY((float) -yaw.getSin())
        .setZ(0)
        .build();

    var position2d = Position3d.newBuilder()
        .setPosition(position)
        .setDirection(direction)
        .build();

    var velocity = Vector3.newBuilder()
        .setX((float) velocityXYZ[0])
        .setY((float) velocityXYZ[1])
        .setZ((float) velocityXYZ[2])
        .build();

    var acceleration = Vector3.newBuilder()
        .setX((float) accelerationXYZ[0])
        .setY((float) accelerationXYZ[1])
        .setZ((float) accelerationXYZ[2])
        .build();

    var angularVel = Vector3.newBuilder().setX((float) angularVelocity[0]).setY((float) angularVelocity[1])
        .setZ((float) angularVelocity[2])
        .build();

    var imuData = ImuData.newBuilder()
        .setPosition(position2d)
        .setVelocity(velocity)
        .setAcceleration(acceleration)
        .setAngularVelocityXYZ(angularVel)
        .build();

    var all = GeneralSensorData.newBuilder().setImu(imuData).setSensorName(SensorName.IMU).setSensorId("0")
        .setTimestamp(System.currentTimeMillis()).setProcessingTimeMs(0);

    return all.build().toByteArray();
  }

  @Override
  public String getPublishTopic() {
    return "imu/imu";
  }
}
