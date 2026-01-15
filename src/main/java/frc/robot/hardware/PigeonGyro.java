package frc.robot.hardware;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CustomMath;
import proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import proto.sensor.GeneralSensorDataOuterClass.SensorName;
import proto.sensor.Imu.ImuData;
import proto.util.Position.Position3d;
import proto.util.Vector.Vector3;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;
import pwrup.frc.core.proto.IDataClass;

public class PigeonGyro extends SubsystemBase implements IGyroscopeLike, IDataClass {
  private static PigeonGyro instance;
  private static int defaultCanId = 0;
  private final Pigeon2 pigeon;
  private double xOffset = 0;
  private double yOffset = 0;
  private double zOffset = 0;
  private double yawSoftOffsetDeg = 0.0;

  public PigeonGyro(int canId) {
    this.pigeon = new Pigeon2(canId);
    pigeon.reset();
    yawSoftOffsetDeg = 0.0;
  }

  /**
   * Set the default CAN ID used by GetInstance().
   * Call this before the first call to GetInstance().
   */
  public static void setDefaultCanId(int canId) {
    defaultCanId = canId;
  }

  public static PigeonGyro GetInstance() {
    if (instance == null) {
      instance = new PigeonGyro(defaultCanId);
    }

    return instance;
  }

  public Pigeon2 getGyro() {
    return pigeon;
  }

  @Override
  public double[] getYPR() {
    double yawAdj = CustomMath.wrapTo180(pigeon.getYaw().getValueAsDouble() + yawSoftOffsetDeg);
    return new double[] {
        yawAdj,
        pigeon.getPitch().getValueAsDouble(),
        pigeon.getRoll().getValueAsDouble(),
    };
  }

  @Override
  public void setPositionAdjustment(double x, double y, double z) {
    xOffset = x;
    yOffset = y;
    zOffset = z;
  }

  @Override
  public double[] getLinearAccelerationXYZ() {
    return new double[] {
        pigeon.getAccelerationX().getValueAsDouble(),
        pigeon.getAccelerationY().getValueAsDouble(),
        pigeon.getAccelerationZ().getValueAsDouble(),
    };
  }

  @Override
  public double[] getAngularVelocityXYZ() {
    return new double[] {
        Math.toRadians(pigeon.getAngularVelocityXWorld().getValueAsDouble()),
        Math.toRadians(pigeon.getAngularVelocityYWorld().getValueAsDouble()),
        Math.toRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble()),
    };
  }

  @Override
  public double[] getQuaternion() {
    return new double[] {
        pigeon.getQuatW().getValueAsDouble(),
        pigeon.getQuatX().getValueAsDouble(),
        pigeon.getQuatY().getValueAsDouble(),
        pigeon.getQuatZ().getValueAsDouble(),
    };
  }

  @Override
  public double[] getLinearVelocityXYZ() {
    // Pigeon2 doesn't provide velocity directly, return zeros
    return new double[] { 0, 0, 0 };
  }

  @Override
  public double[] getPoseXYZ() {
    // Pigeon2 doesn't provide displacement directly, return offsets
    return new double[] {
        xOffset,
        yOffset,
        zOffset,
    };
  }

  @Override
  public void reset() {
    pigeon.reset();
    yawSoftOffsetDeg = 0.0;
  }

  @Override
  public void setAngleAdjustment(double angle) {
    pigeon.setYaw(0);
    yawSoftOffsetDeg = -angle;
  }

  public void setYawDeg(double targetDeg) {
    setAngleAdjustment(targetDeg);
  }

  public Rotation2d getNoncontinuousAngle() {
    return Rotation2d.fromDegrees(CustomMath.wrapTo180(pigeon.getYaw().getValueAsDouble()));
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
    Logger.recordOutput("PigeonGyro/yaw", getYPR()[0]);
  }
}
