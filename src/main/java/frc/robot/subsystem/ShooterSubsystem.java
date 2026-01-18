package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.ShooterConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem instance;

  private final SparkMax m_shooterMotor;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder relativeEncoder;

  private LinearVelocity lastShooterVelocitySetpoint;

  public static ShooterSubsystem GetInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem(ShooterConstants.kShooterCanId, ShooterConstants.kShooterMotorType);
    }

    return instance;
  }

  public ShooterSubsystem(int canId, MotorType motorType) {
    this.m_shooterMotor = new SparkMax(canId, motorType);
    this.closedLoopController = m_shooterMotor.getClosedLoopController();
    this.relativeEncoder = m_shooterMotor.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(ShooterConstants.kShooterReversed)
        .smartCurrentLimit(ShooterConstants.kShooterCurrentLimit);

    double factor = (2 * ShooterConstants.kWheelRadius * Math.PI)
        / ShooterConstants.kShooterMotorRotationsPerRotation;

    config.encoder.velocityConversionFactor(factor / 60.0);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD)
        .iZone(ShooterConstants.kShooterIZ);

    config.closedLoop.maxMotion
        .cruiseVelocity(ShooterConstants.kShooterMaxVelocity.in(Units.MetersPerSecond))
        .maxAcceleration(ShooterConstants.kShooterMaxAcceleration.in(Units.MetersPerSecondPerSecond));

    m_shooterMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * Set the shooter velocity in meters per second.
   * 
   * @param velocity The velocity to set the shooter to.
   * @return the time in ms it will take to reach the velocity
   **/
  public int setShooterVelocity(LinearVelocity velocity) {
    lastShooterVelocitySetpoint = velocity;
    closedLoopController.setSetpoint(velocity.in(Units.MetersPerSecond), ControlType.kMAXMotionVelocityControl);
    return timeLeftToReachVelocity();
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

    closedLoopController.setSetpoint(lastShooterVelocitySetpoint.in(Units.MetersPerSecond),
        ControlType.kMAXMotionVelocityControl);
    return timeLeftToReachVelocity();
  }

  /**
   * Estimates the time (in milliseconds) to reach the provided shooter velocity.
   * Returns 0 if target velocity is already achieved or if acceleration is
   * non-positive.
   */
  public int timeLeftToReachVelocity(LinearVelocity velocity) {
    double currentVelocityMps = getCurrentShooterVelocity().in(Units.MetersPerSecond);
    double targetVelocityMps = velocity.in(Units.MetersPerSecond);
    double acceleration = ShooterConstants.kShooterMaxAcceleration.in(Units.MetersPerSecondPerSecond);

    double velocityDelta = Math.max(0, Math.abs(targetVelocityMps - currentVelocityMps));
    if (acceleration <= 0)
      return 0;

    double seconds = velocityDelta / acceleration;
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
   * Get the current shooter velocity in meters per second.
   * 
   * @return the current shooter velocity
   **/
  public LinearVelocity getCurrentShooterVelocity() {
    return Units.MetersPerSecond.of(relativeEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/Velocity", getCurrentShooterVelocity().in(Units.MetersPerSecond));
    Logger.recordOutput("Shooter/Position", Units.Meters.of(relativeEncoder.getPosition()));
    Logger.recordOutput("Shooter/TimeLeftToReachVelocity", timeLeftToReachVelocity());
  }
}
