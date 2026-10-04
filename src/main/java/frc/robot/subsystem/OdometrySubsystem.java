package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.UnifiedGyro;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;

public class OdometrySubsystem extends SubsystemBase {

  private static OdometrySubsystem self;
  private final SwerveSubsystem swerve;
  private final SwerveDriveOdometry odometry;
  private final IGyroscopeLike gyro;
  public Pose2d[] timedPositions = new Pose2d[] { new Pose2d(), new Pose2d() };
  public long[] timestamps = new long[2];

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
    timestamps[0] = System.currentTimeMillis();
    timestamps[1] = timestamps[0];
  }

  public void setOdometryPosition(Pose2d newPose) {
    SwerveModulePosition[] currentModulePositions = copyModulePositions(swerve.getSwerveModulePositions());
    odometry.resetPosition(
        gyro.getRotation2d(),
        currentModulePositions,
        newPose);
    timedPositions[0] = newPose;
    timedPositions[1] = newPose;
    timestamps[0] = System.currentTimeMillis();
    timestamps[1] = timestamps[0];
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

  @Override
  public void periodic() {
    timedPositions[0] = timedPositions[1];
    timestamps[0] = timestamps[1];
    timestamps[1] = System.currentTimeMillis();

    var positions = swerve.getSwerveModulePositions();
    timedPositions[1] = odometry.update(gyro.getRotation2d(), positions);

    Logger.recordOutput("Odometry/Position", timedPositions[1]);
    Logger.recordOutput("Odometry/Velocity", swerve.getChassisSpeeds());
  }
}
