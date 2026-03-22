package frc.robot.subsystem;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.pwrup.SwerveDrive;
import org.pwrup.util.Config;
import org.pwrup.util.Vec2;
import org.pwrup.util.Wheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.BotConstants;
import frc.robot.constant.BotConstants.RobotVariant;
import frc.robot.constant.swerve.SwerveConstants;
import frc.robot.hardware.UnifiedGyro;
import frc.robot.hardware.WheelMoverBase;
import frc.robot.hardware.WheelMoverSpark;
import frc.robot.hardware.WheelMoverTalonFX;
import frc.robot.util.LocalMath;
import lombok.Getter;
import lombok.Setter;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;

public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem self;

  public enum DriveType {
    FIELD_RELATIVE,
    RAW,
    DRIVER_RELATIVE,
  }

  private Rotation2d swerveRotationOffset;

  public final WheelMoverBase m_frontLeftSwerveModule;
  private final WheelMoverBase m_frontRightSwerveModule;
  private final WheelMoverBase m_rearLeftSwerveModule;
  private final WheelMoverBase m_rearRightSwerveModule;

  private final SwerveDrive swerve;
  private final IGyroscopeLike m_gyro;
  private final SwerveDriveKinematics kinematics;

  @Setter
  private boolean shouldWork = true;
  @Getter
  private boolean isGpsAssist = false;

  public static SwerveSubsystem GetInstance() {
    return GetInstance(UnifiedGyro.GetInstance());
  }

  public static SwerveSubsystem GetInstance(IGyroscopeLike gyro) {
    if (self == null) {
      self = new SwerveSubsystem(gyro);
    }

    return self;
  }

  public SwerveSubsystem(IGyroscopeLike gyro) {
    this.m_gyro = gyro;
    this.swerveRotationOffset = new Rotation2d();
    final var c = SwerveConstants.INSTANCE;
    this.isGpsAssist = true;

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

  public void drive(ChassisSpeeds speeds, DriveType driveType) {
    if (!shouldWork) {
      stop();
      return;
    }

    switch (driveType) {
      case FIELD_RELATIVE:
        driveFieldRelative(speeds);
        break;
      case DRIVER_RELATIVE:
        driveDriverRelative(speeds);
        break;
      case RAW:
        driveRaw(speeds);
        break;
      default:
        driveRaw(speeds);
        break;
    }
  }

  public void stop() {
    driveRaw(new ChassisSpeeds(0, 0, 0));
  }

  private void driveRaw(ChassisSpeeds speeds) {
    swerve.driveNonRelative(speeds);
  }

  private void driveFieldRelative(ChassisSpeeds speeds) {
    swerve.driveWithGyro(speeds, getFieldRelativeRotation());
  }

  private void driveDriverRelative(ChassisSpeeds speeds) {
    swerve.driveWithGyro(speeds, getSwerveRotationWithOffset());
  }

  private Rotation2d getFieldRelativeRotation() {
    return m_gyro.getRotation().toRotation2d();
  }

  private Rotation2d getSwerveRotationWithOffset() {
    return swerveRotationOffset.minus(getFieldRelativeRotation());
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeftSwerveModule.getPosition(),
        m_frontRightSwerveModule.getPosition(),
        m_rearLeftSwerveModule.getPosition(),
        m_rearRightSwerveModule.getPosition(),
    };
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

  public void resetDriverRelative() {
    resetDriverRelative(getFieldRelativeRotation());
  }

  public void resetDriverRelative(Rotation2d newCur) {
    swerveRotationOffset = newCur;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("SwerveSubsystem/swerve/states", getSwerveModuleStates());
    Logger.recordOutput("SwerveSubsystem/swerve/velocity", getKinematics().toChassisSpeeds(getSwerveModuleStates()));
    Logger.recordOutput("SwerveSubsystem/AdjustingVelocity", isGpsAssist);
  }

  public static ChassisSpeeds fromPercentToVelocity(Vec2 percentXY, double rotationPercent) {
    double vx = LocalMath.clamp(percentXY.getX(), -1, 1) * SwerveConstants.kRobotMaxSpeed.in(Units.MetersPerSecond);
    double vy = LocalMath.clamp(percentXY.getY(), -1, 1) * SwerveConstants.kRobotMaxSpeed.in(Units.MetersPerSecond);
    double omega = LocalMath.clamp(rotationPercent, -1, 1)
        * SwerveConstants.kRobotMaxTurnSpeed.in(Units.RadiansPerSecond);
    return new ChassisSpeeds(vx, vy, omega);
  }

  public void setGpsAssist(boolean isGpsAssist) {
    this.isGpsAssist = isGpsAssist;
  }
}
