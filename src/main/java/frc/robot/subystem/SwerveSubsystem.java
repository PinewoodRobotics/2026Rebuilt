package frc.robot.subystem;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.pwrup.SwerveDrive;
import org.pwrup.util.Config;
import org.pwrup.util.Vec2;
import org.pwrup.util.Wheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.BotConstants;
import frc.robot.constant.BotConstants.RobotVariant;
import frc.robot.constant.swerve.SwerveConstants;
import frc.robot.hardware.AHRSGyro;
import frc.robot.hardware.WheelMoverBase;
import frc.robot.hardware.WheelMoverSpark;
import frc.robot.hardware.WheelMoverTalonFX;
import frc.robot.util.CustomMath;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;

/**
 * Minimal swerve subsystem: drives with joystick input through PWRUP SwerveDrive.
 */
public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem self;
  public final WheelMoverBase m_frontLeftSwerveModule;
  private final WheelMoverBase m_frontRightSwerveModule;
  private final WheelMoverBase m_rearLeftSwerveModule;
  private final WheelMoverBase m_rearRightSwerveModule;

  private final SwerveDrive swerve;
  private final IGyroscopeLike m_gyro;
  private double gyroOffset = 0;
  private boolean shouldWork = true;

  private final SwerveDriveKinematics kinematics;

  public static SwerveSubsystem GetInstance() {
    if (self == null) {
      self = new SwerveSubsystem(AHRSGyro.GetInstance());
    }

    return self;
  }

  public SwerveSubsystem(IGyroscopeLike gyro) {
    this.m_gyro = gyro;
    final var c = SwerveConstants.INSTANCE;

    if (BotConstants.robotType == RobotVariant.BBOT) {
      this.m_frontLeftSwerveModule = new WheelMoverSpark(
          c.kFrontLeftDriveMotorPort,
          c.kFrontLeftDriveMotorReversed,
          c.kFrontLeftTurningMotorPort,
          c.kFrontLeftTurningMotorReversed,
          c.kFrontLeftCANcoderPort,
          c.kFrontLeftCANcoderDirection,
          c.kFrontLeftCANcoderMagnetOffset);
      this.m_frontRightSwerveModule = new WheelMoverSpark(
          c.kFrontRightDriveMotorPort,
          c.kFrontRightDriveMotorReversed,
          c.kFrontRightTurningMotorPort,
          c.kFrontRightTurningMotorReversed,
          c.kFrontRightCANcoderPort,
          c.kFrontRightCANcoderDirection,
          c.kFrontRightCANcoderMagnetOffset);
      this.m_rearLeftSwerveModule = new WheelMoverSpark(
          c.kRearLeftDriveMotorPort,
          c.kRearLeftDriveMotorReversed,
          c.kRearLeftTurningMotorPort,
          c.kRearLeftTurningMotorReversed,
          c.kRearLeftCANcoderPort,
          c.kRearLeftCANcoderDirection,
          c.kRearLeftCANcoderMagnetOffset);
      this.m_rearRightSwerveModule = new WheelMoverSpark(
          c.kRearRightDriveMotorPort,
          c.kRearRightDriveMotorReversed,
          c.kRearRightTurningMotorPort,
          c.kRearRightTurningMotorReversed,
          c.kRearRightCANcoderPort,
          c.kRearRightCANcoderDirection,
          c.kRearRightCANcoderMagnetOffset);
    } else {
      this.m_frontLeftSwerveModule = new WheelMoverTalonFX(
          c.kFrontLeftDriveMotorPort,
          c.kFrontLeftDriveMotorReversed,
          c.kFrontLeftTurningMotorPort,
          c.kFrontLeftTurningMotorReversed,
          c.kFrontLeftCANcoderPort,
          c.kFrontLeftCANcoderDirection,
          c.kFrontLeftCANcoderMagnetOffset);
      this.m_frontRightSwerveModule = new WheelMoverTalonFX(
          c.kFrontRightDriveMotorPort,
          c.kFrontRightDriveMotorReversed,
          c.kFrontRightTurningMotorPort,
          c.kFrontRightTurningMotorReversed,
          c.kFrontRightCANcoderPort,
          c.kFrontRightCANcoderDirection,
          c.kFrontRightCANcoderMagnetOffset);
      this.m_rearLeftSwerveModule = new WheelMoverTalonFX(
          c.kRearLeftDriveMotorPort,
          c.kRearLeftDriveMotorReversed,
          c.kRearLeftTurningMotorPort,
          c.kRearLeftTurningMotorReversed,
          c.kRearLeftCANcoderPort,
          c.kRearLeftCANcoderDirection,
          c.kRearLeftCANcoderMagnetOffset);
      this.m_rearRightSwerveModule = new WheelMoverTalonFX(
          c.kRearRightDriveMotorPort,
          c.kRearRightDriveMotorReversed,
          c.kRearRightTurningMotorPort,
          c.kRearRightTurningMotorReversed,
          c.kRearRightCANcoderPort,
          c.kRearRightCANcoderDirection,
          c.kRearRightCANcoderMagnetOffset);
    }

    this.swerve = new SwerveDrive(
        new Config(
            Optional.empty(),
            new Wheel[] {
                new Wheel(
                    c.frontRightTranslation,
                    m_frontRightSwerveModule),
                new Wheel(
                    c.frontLeftTranslation,
                    m_frontLeftSwerveModule),
                new Wheel(
                    c.rearLeftTranslation,
                    m_rearLeftSwerveModule),
                new Wheel(
                    c.rearRightTranslation,
                    m_rearRightSwerveModule),
            }));

    this.kinematics = new SwerveDriveKinematics(
        c.frontLeftTranslation,
        c.frontRightTranslation,
        c.rearLeftTranslation,
        c.rearRightTranslation);
  }

  public void stop() {
    driveRaw(new ChassisSpeeds(0, 0, 0));
  }

  public enum DriveType {
    GYRO_RELATIVE,
    RAW,
  }

  public void drive(ChassisSpeeds speeds, DriveType driveType) {
    if (!shouldWork) {
      stop();
      return;
    }

    switch (driveType) {
      case GYRO_RELATIVE:
        driveFieldRelative(speeds);
        break;
      case RAW:
        driveRaw(speeds);
        break;
      default:
        driveRaw(speeds);
        break;
    }
  }

  public void driveRaw(ChassisSpeeds speeds) {
    var actualSpeeds = toSwerveOrientation(speeds);
    swerve.driveNonRelative(actualSpeeds);
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    var actualSpeeds = toSwerveOrientation(speeds);
    swerve.driveWithGyro(actualSpeeds, new Rotation2d(getSwerveGyroAngle()));
  }

  public static ChassisSpeeds fromPercentToVelocity(Vec2 percentXY, double rotationPercent) {
    final var c = SwerveConstants.INSTANCE;
    double vx = clamp(percentXY.getX(), -1, 1) * c.tempMaxSpeed;
    double vy = clamp(percentXY.getY(), -1, 1) * c.tempMaxSpeed;
    double omega = clamp(rotationPercent, -1, 1) * c.kMaxAngularSpeedRadPerSec;
    return new ChassisSpeeds(vx, vy, omega);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeftSwerveModule.getPosition(),
        m_frontRightSwerveModule.getPosition(),
        m_rearLeftSwerveModule.getPosition(),
        m_rearRightSwerveModule.getPosition(),
    };
  }

  public ChassisSpeeds getGlobalChassisSpeeds(Rotation2d heading) {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), heading);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public SwerveDriveKinematics getKinematics() {
    return this.kinematics;
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeftSwerveModule.getState(),
        m_frontRightSwerveModule.getState(),
        m_rearLeftSwerveModule.getState(),
        m_rearRightSwerveModule.getState(),
    };
  }

  public void resetGyro() {
    resetGyro(0);
  }

  public void resetGyro(double offset) {
    gyroOffset = -m_gyro.getYaw() + offset;
  }

  public double getSwerveGyroAngle() {
    return Math.toRadians(CustomMath.wrapTo180(m_gyro.getYaw() + gyroOffset));
  }

  public void setShouldWork(boolean value) {
    this.shouldWork = value;
    if (!shouldWork) {
      stop(); // make sure it applies immediately
    }
  }

  private static double clamp(double v, double min, double max) {
    return Math.max(min, Math.min(max, v));
  }

  private static ChassisSpeeds toSwerveOrientation(ChassisSpeeds target) {
    return new ChassisSpeeds(
        -target.vxMetersPerSecond,
        target.vyMetersPerSecond,
        target.omegaRadiansPerSecond);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("SwerveSubsystem/swerve/states", getSwerveModuleStates());
  }
}
