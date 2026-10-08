package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.UnifiedGyro;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;

public class OdometrySubsystem extends SubsystemBase {

  private static OdometrySubsystem self;
  private final SwerveSubsystem swerve;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final IGyroscopeLike gyro;
  private boolean anchoredToField = false;
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
    this.poseEstimator = new SwerveDrivePoseEstimator(
        swerve.getKinematics(),
        gyro.getRotation2d(),
        initialModulePositions,
        initialPose);
    timedPositions[0] = initialPose;
    timedPositions[1] = initialPose;
    timestamps[0] = System.currentTimeMillis();
    timestamps[1] = timestamps[0];
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public boolean isAnchoredToField() {
    return anchoredToField;
  }

  public long getLastUpdateTimeMs() {
    return timestamps[1];
  }

  public void resetPose(Pose2d newPose) {
    gyro.resetRotation(new Rotation3d(newPose.getRotation()));
    SwerveModulePosition[] currentModulePositions = copyModulePositions(swerve.getSwerveModulePositions());
    poseEstimator.resetPosition(
        gyro.getRotation2d(),
        currentModulePositions,
        newPose);
    anchoredToField = true;
    timedPositions[0] = newPose;
    timedPositions[1] = newPose;
    timestamps[0] = System.currentTimeMillis();
    timestamps[1] = timestamps[0];
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
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
    timedPositions[1] = poseEstimator.update(gyro.getRotation2d(), positions);

    Logger.recordOutput("Odometry/Position", timedPositions[1]);
    Logger.recordOutput("Odometry/Velocity", swerve.getChassisSpeeds());
    Logger.recordOutput("Odometry/AnchoredToField", anchoredToField);
  }
}
