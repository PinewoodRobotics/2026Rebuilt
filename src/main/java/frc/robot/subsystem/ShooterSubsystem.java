package frc.robot.subsystem;

import frc.robot.constant.BotConstants;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.wpilib.units.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.command2.SubsystemBase;
import frc.robot.constant.ShooterConstants;
import lombok.Setter;

public class ShooterSubsystem extends SubsystemBase {
  private static final double kStopVelocityThresholdRpm = 1e-3;
  private static final double kMechanismRpmPerMotorRpm = 1.0 / ShooterConstants.kShooterMotorRotationsPerRotation;
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
    this.leaderMotor = new SparkFlex(BotConstants.kCanPort, leaderCanId, leaderMotorType);
    this.followerMotor = new SparkFlex(BotConstants.kCanPort, followerCanId, followerMotorType);

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

    // REVLib 2027 has no encoder conversion factors, so the Spark closed loop runs in motor RPM.
    // Scaling the gains by the ratio keeps them equivalent to gains tuned in mechanism RPM.
    double k = kMechanismRpmPerMotorRpm;
    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.kShooterP * k, ShooterConstants.kShooterI * k, ShooterConstants.kShooterD * k)
        .iZone(ShooterConstants.kShooterIZ / k);
    followerConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.kShooterFollowerP * k, ShooterConstants.kShooterI * k, ShooterConstants.kShooterD * k)
        .iZone(ShooterConstants.kShooterIZ / k);

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
    double targetMotorRpm = targetRpm / kMechanismRpmPerMotorRpm;
    leaderClosedLoopController.setSetpoint(targetMotorRpm, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, feedForwardLeader);

    double feedForwardFollower = ShooterConstants.kFFFollower * targetRpm;
    followerClosedLoopController.setSetpoint(targetMotorRpm, ControlType.kVelocity,
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
      return Math.abs(getLeaderVelocityRpm()) <= kStopVelocityThresholdRpm
          && Math.abs(getFollowerVelocityRpm()) <= kStopVelocityThresholdRpm;
    }

    double velocityToleranceRpm = ShooterConstants.kShooterVelocityTolerance.in(Units.RPM);
    double leaderVelocityError = Math.abs(targetVelocityRpm - getLeaderVelocityRpm());
    double followerVelocityError = Math.abs(targetVelocityRpm - getFollowerVelocityRpm());

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
    return Units.RPM.of((getLeaderVelocityRpm() + getFollowerVelocityRpm()) / 2.0);
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

  private double getLeaderVelocityRpm() {
    return leaderEncoder.getVelocity().get() * kMechanismRpmPerMotorRpm;
  }

  private double getFollowerVelocityRpm() {
    return followerEncoder.getVelocity().get() * kMechanismRpmPerMotorRpm;
  }

  @Override
  public void periodic() {
    double currentLeaderVelocityRpm = getLeaderVelocityRpm();
    double currentFollowerVelocityRpm = getFollowerVelocityRpm();

    Logger.recordOutput("Shooter/VelocityRPM", getCurrentShooterVelocity().in(Units.RPM));
    Logger.recordOutput("Shooter/LeaderVelocityRPM", currentLeaderVelocityRpm);
    Logger.recordOutput("Shooter/FollowerVelocityRPM", currentFollowerVelocityRpm);
    Logger.recordOutput("Shooter/RequestedVelocityRPM",
        lastShooterVelocitySetpoint == null ? 0.0 : lastShooterVelocitySetpoint.in(Units.RPM));
    Logger.recordOutput("Shooter/IsSpunUp", isShooterSpunUp());
    Logger.recordOutput("Shooter/LeaderAppliedOutput", leaderMotor.getAppliedOutput().get());
    Logger.recordOutput("Shooter/FollowerAppliedOutput", followerMotor.getAppliedOutput().get());
    Logger.recordOutput("Shooter/LeaderPositionRotations", Units.Rotations.of(leaderEncoder.getPosition().get()));
    Logger.recordOutput("Shooter/FollowerPositionRotations", Units.Rotations.of(followerEncoder.getPosition().get()));
  }
}
