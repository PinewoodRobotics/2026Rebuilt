package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.constant.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  private static TurretSubsystem instance;

  private SparkFlex m_turretMotor;
  private SparkClosedLoopController closedLoopController;
  private final RelativeEncoder relativeEncoder;

  /** Last commanded turret goal angle (for logging / time estimate). */
  private Angle lastAimTarget;

  public static TurretSubsystem GetInstance() {
    if (instance == null) {
      instance = new TurretSubsystem(TurretConstants.kTurretCanId, TurretConstants.kTurretMotorType);
    }

    return instance;
  }

  public TurretSubsystem(int canId, MotorType motorType) {
    configureSparkMax(canId, motorType);
    relativeEncoder = m_turretMotor.getEncoder();
    reset();
  }

  private void configureSparkMax(int canId, MotorType motorType) {
    this.m_turretMotor = new SparkFlex(canId, motorType);
    this.closedLoopController = m_turretMotor.getClosedLoopController();

    SparkFlexConfig config = new SparkFlexConfig();

    config.idleMode(IdleMode.kBrake);
    config.inverted(TurretConstants.kMotorInverted);

    config
        .smartCurrentLimit(TurretConstants.kTurretCurrentLimit);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD)
        .iZone(TurretConstants.kTurretIZ)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(0)
        .positionWrappingMaxInput(1);

    m_turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void reset() {
    relativeEncoder.setPosition(0.0);
  }

  /**
   * Simple position PID (no MAXMotion).
   */
  public void setTurretPosition(Angle position, Voltage feedForward) {
    lastAimTarget = position;

    closedLoopController.setSetpoint(
        position.in(Units.Rotations),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedForward.in(Units.Volts),
        ArbFFUnits.kVoltage);
  }

  public int getAimTimeLeftMs() {
    if (lastAimTarget == null) {
      return 0;
    }

    double maxVelRotPerSec = TurretConstants.kTurretMaxVelocity.in(Units.RotationsPerSecond);
    if (maxVelRotPerSec <= 0.0) {
      return 0;
    }

    double currentRot = getTurretPosition().in(Units.Rotations);
    double targetRot = lastAimTarget.in(Units.Rotations);

    // shortest-path wrapped error (-0.5 .. 0.5 rotations)
    double errorRot = targetRot - currentRot;
    errorRot = errorRot - Math.floor(errorRot + 0.5);

    double distanceRot = Math.abs(errorRot);

    double timeSec = distanceRot / maxVelRotPerSec;

    return (int) Math.ceil(timeSec * 1000.0);
  }

  public Angle getTurretPosition() {
    return Units.Rotations.of(m_turretMotor.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Turret/PositionRot", getTurretPosition().in(Units.Rotations));
    Logger.recordOutput("Turret/PositionDeg", getTurretPosition().in(Units.Degrees));
    Logger.recordOutput("Turret/Velocity", m_turretMotor.getEncoder().getVelocity());
    Logger.recordOutput("Turret/DesiredOutputRot", lastAimTarget != null ? lastAimTarget.in(Units.Rotations) : 0);
    Logger.recordOutput("Turret/AppliedOutput", m_turretMotor.getAppliedOutput());
    Logger.recordOutput("Turret/BusVoltage", m_turretMotor.getBusVoltage());
    Logger.recordOutput("Turret/TimeTillGoal", getAimTimeLeftMs());
  }
}
