package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private static final double kStopVelocityThresholdRpm = 1e-3;
  private static ShooterSubsystem instance;

  private final SparkMax leaderMotor, followerMotor;
  private final SparkClosedLoopController leaderClosedLoopController, followerClosedLoopController;
  private final RelativeEncoder leaderEncoder, followerEncoder;

  private AngularVelocity lastShooterVelocitySetpoint;

  public static ShooterSubsystem GetInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem(ShooterConstants.kShooterCanId, ShooterConstants.kShooterMotorType,
          ShooterConstants.kShooterCanIdFollower, ShooterConstants.kShooterMotorTypeFollower);
    }

    return instance;
  }

  public ShooterSubsystem(
      int leaderCanId,
      MotorType leaderMotorType,
      int followerCanId,
      MotorType followerMotorType) {
    this.leaderMotor = new SparkMax(leaderCanId, leaderMotorType);
    this.followerMotor = new SparkMax(followerCanId, followerMotorType);

    this.leaderClosedLoopController = leaderMotor.getClosedLoopController();
    this.followerClosedLoopController = followerMotor.getClosedLoopController();
    this.leaderEncoder = leaderMotor.getEncoder();
    this.followerEncoder = followerMotor.getEncoder();

    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .inverted(ShooterConstants.kShooterLeaderReversed)
        .smartCurrentLimit(ShooterConstants.kShooterCurrentLimit)
        .idleMode(IdleMode.kCoast);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .inverted(ShooterConstants.kShooterFollowerReversed)
        .smartCurrentLimit(ShooterConstants.kShooterCurrentLimit)
        .idleMode(IdleMode.kCoast);

    double mechanismRpmPerMotorRpm = 1.0 / ShooterConstants.kShooterMotorRotationsPerRotation;

    // Spark MAX native velocity is RPM; keep encoder values in mechanism RPM.
    leaderConfig.encoder.velocityConversionFactor(mechanismRpmPerMotorRpm);
    followerConfig.encoder.velocityConversionFactor(mechanismRpmPerMotorRpm);

    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD)
        .iZone(ShooterConstants.kShooterIZ);
    followerConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.kShooterFollowerP, ShooterConstants.kShooterI, ShooterConstants.kShooterD)
        .iZone(ShooterConstants.kShooterIZ);

    leaderMotor.configure(
        leaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    followerMotor.configure(
        followerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * Set the shooter velocity in RPM.
   * 
   * @param velocity The velocity to set the shooter to.
   * @return the time in ms it will take to reach the velocity
   **/
  public int setShooterVelocity(AngularVelocity velocity) {
    lastShooterVelocitySetpoint = velocity;
    double targetRpm = velocity.in(Units.RPM);

    if (Math.abs(targetRpm) <= kStopVelocityThresholdRpm) {
      stopShooter();
      return 0;
    }

    double feedForward = ShooterConstants.kFF * targetRpm;
    leaderClosedLoopController.setSetpoint(targetRpm, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, feedForward);
    followerClosedLoopController.setSetpoint(targetRpm, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, feedForward);

    return timeLeftToReachVelocity();
  }

  public void stopShooter() {
    leaderMotor.stopMotor();
    followerMotor.stopMotor();
  }

  /**
   * Re-issues the most recently commanded shooter velocity setpoint (if any).
   *
   * @return the time in ms it will take to reach the last setpoint (0 if none)
   */
  public int setShooterVelocity() {
    if (lastShooterVelocitySetpoint == null) {
      return 0;
    }

    return setShooterVelocity(lastShooterVelocitySetpoint);
  }

  public void runMotorBaseSpeed() {
    setShooterVelocity(ShooterConstants.kShooterBaseSpeed);
  }

  /**
   * Estimates the time (in milliseconds) to reach the provided shooter velocity.
   * Returns 0 if target velocity is already achieved or if acceleration is
   * non-positive.
   */
  public int timeLeftToReachVelocity(AngularVelocity velocity) {
    double targetVelocityRpm = velocity.in(Units.RPM);
    double accelerationRpmPerSecond = ShooterConstants.kShooterMaxAcceleration.in(Units.RotationsPerSecondPerSecond)
        * 60.0;

    double leaderVelocityDelta = Math.abs(targetVelocityRpm - leaderEncoder.getVelocity());
    double followerVelocityDelta = Math.abs(targetVelocityRpm - followerEncoder.getVelocity());
    double velocityDelta = Math.max(leaderVelocityDelta, followerVelocityDelta);
    if (accelerationRpmPerSecond <= 0)
      return 0;

    double seconds = velocityDelta / accelerationRpmPerSecond;
    return (int) Math.ceil(seconds * 1000.0);
  }

  /**
   * Estimates the time (in milliseconds) to reach the most recently commanded
   * shooter velocity setpoint. Returns 0 if no setpoint has been commanded yet.
   */
  public int timeLeftToReachVelocity() {
    if (lastShooterVelocitySetpoint == null) {
      return 0;
    }

    return timeLeftToReachVelocity(lastShooterVelocitySetpoint);
  }

  /**
   * Get the current shooter velocity in RPM.
   * 
   * @return the current shooter velocity
   **/
  public AngularVelocity getCurrentShooterVelocity() {
    return Units.RPM.of((leaderEncoder.getVelocity() + followerEncoder.getVelocity()) / 2.0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/VelocityRPM", getCurrentShooterVelocity().in(Units.RPM));
    Logger.recordOutput("Shooter/LeaderVelocityRPM", leaderEncoder.getVelocity());
    Logger.recordOutput("Shooter/FollowerVelocityRPM", followerEncoder.getVelocity());
    Logger.recordOutput("Shooter/RequestedVelocityRPM",
        lastShooterVelocitySetpoint == null ? 0.0 : lastShooterVelocitySetpoint.in(Units.RPM));
    Logger.recordOutput("Shooter/LeaderAppliedOutput", leaderMotor.getAppliedOutput());
    Logger.recordOutput("Shooter/FollowerAppliedOutput", followerMotor.getAppliedOutput());
    Logger.recordOutput("Shooter/LeaderPositionRotations", Units.Rotations.of(leaderEncoder.getPosition()));
    Logger.recordOutput("Shooter/FollowerPositionRotations", Units.Rotations.of(followerEncoder.getPosition()));
    Logger.recordOutput("Shooter/TimeLeftToReachVelocity", timeLeftToReachVelocity());
  }
}
