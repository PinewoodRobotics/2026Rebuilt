package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.ShooterConstants;
import lombok.Setter;

public class ShooterSubsystem extends SubsystemBase {
  private static final double kStopVelocityThresholdRpm = 1e-3;
  private static ShooterSubsystem instance;

  private final SparkFlex leaderMotor, followerMotor;
  private final SparkClosedLoopController leaderClosedLoopController, followerClosedLoopController;
  private final RelativeEncoder leaderEncoder, followerEncoder;

  private AngularVelocity lastShooterVelocitySetpoint;

  @Setter
  private static boolean isGpsAssistEnabled = true;

  public static boolean getIsGpsAssistEnabled() {
    return isGpsAssistEnabled;
  }

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
    this.leaderMotor = new SparkFlex(leaderCanId, leaderMotorType);
    this.followerMotor = new SparkFlex(followerCanId, followerMotorType);

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
   **/
  public void setShooterVelocity(AngularVelocity velocity) {
    lastShooterVelocitySetpoint = velocity;
    if (velocity.in(Units.RotationsPerSecond) > ShooterConstants.kShooterMaxVelocity.in(Units.RotationsPerSecond)) {
      lastShooterVelocitySetpoint = ShooterConstants.kShooterMaxVelocity;
    }

    double targetRpm = velocity.in(Units.RPM);

    if (Math.abs(targetRpm) <= kStopVelocityThresholdRpm) {
      stopShooter();
      return;
    }

    double feedForwardLeader = ShooterConstants.kFFLeader * targetRpm;
    leaderClosedLoopController.setSetpoint(targetRpm, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, feedForwardLeader);

    double feedForwardFollower = ShooterConstants.kFFFollower * targetRpm;
    followerClosedLoopController.setSetpoint(targetRpm, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, feedForwardFollower);
  }

  public void stopShooter() {
    leaderMotor.stopMotor();
    followerMotor.stopMotor();
  }

  /**
   * Re-issues the most recently commanded shooter velocity setpoint (if any).
   */
  public void setShooterVelocity() {
    if (lastShooterVelocitySetpoint == null) {
      return;
    }

    setShooterVelocity(lastShooterVelocitySetpoint);
  }

  public void runMotorBaseSpeed() {
    setShooterVelocity(ShooterConstants.kShooterBaseSpeed);
  }

  public boolean isShooterSpunUp(AngularVelocity velocity) {
    double targetVelocityRpm = velocity.in(Units.RPM);
    if (Math.abs(targetVelocityRpm) <= kStopVelocityThresholdRpm) {
      return Math.abs(leaderEncoder.getVelocity()) <= kStopVelocityThresholdRpm
          && Math.abs(followerEncoder.getVelocity()) <= kStopVelocityThresholdRpm;
    }

    double velocityToleranceRpm = ShooterConstants.kShooterVelocityTolerance.in(Units.RPM);
    double leaderVelocityError = Math.abs(targetVelocityRpm - leaderEncoder.getVelocity());
    double followerVelocityError = Math.abs(targetVelocityRpm - followerEncoder.getVelocity());

    return leaderVelocityError <= velocityToleranceRpm
        && followerVelocityError <= velocityToleranceRpm;
  }

  public boolean isShooterSpunUp() {
    if (lastShooterVelocitySetpoint == null) {
      return false;
    }

    return isShooterSpunUp(lastShooterVelocitySetpoint);
  }

  /**
   * Get the current shooter velocity in RPM.
   * 
   * @return the current shooter velocity
   **/
  public AngularVelocity getCurrentShooterVelocity() {
    return Units.RPM.of((leaderEncoder.getVelocity() + followerEncoder.getVelocity()) / 2.0);
  }

  public double getCurrentShooterVelocityRps() {
    return getCurrentShooterVelocity().in(Units.RotationsPerSecond);
  }

  public double getRequestedShooterVelocityRps() {
    if (lastShooterVelocitySetpoint == null) {
      return 0.0;
    }
    return lastShooterVelocitySetpoint.in(Units.RotationsPerSecond);
  }

  @Override
  public void periodic() {
    double currentLeaderVelocityRpm = leaderEncoder.getVelocity();
    double currentFollowerVelocityRpm = followerEncoder.getVelocity();

    Logger.recordOutput("Shooter/VelocityRPM", getCurrentShooterVelocity().in(Units.RPM));
    Logger.recordOutput("Shooter/LeaderVelocityRPM", currentLeaderVelocityRpm);
    Logger.recordOutput("Shooter/FollowerVelocityRPM", currentFollowerVelocityRpm);
    Logger.recordOutput("Shooter/RequestedVelocityRPM",
        lastShooterVelocitySetpoint == null ? 0.0 : lastShooterVelocitySetpoint.in(Units.RPM));
    Logger.recordOutput("Shooter/IsSpunUp", isShooterSpunUp());
    Logger.recordOutput("Shooter/LeaderAppliedOutput", leaderMotor.getAppliedOutput());
    Logger.recordOutput("Shooter/FollowerAppliedOutput", followerMotor.getAppliedOutput());
    Logger.recordOutput("Shooter/LeaderPositionRotations", Units.Rotations.of(leaderEncoder.getPosition()));
    Logger.recordOutput("Shooter/FollowerPositionRotations", Units.Rotations.of(followerEncoder.getPosition()));
  }
}
