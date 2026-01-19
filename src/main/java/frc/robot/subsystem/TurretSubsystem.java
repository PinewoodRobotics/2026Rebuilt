package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constant.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  private static TurretSubsystem instance;

  private SparkMax m_turretMotor;
  private SparkClosedLoopController closedLoopController;

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
  }

  private void configureSparkMax(int canId, MotorType motorType) {
    this.m_turretMotor = new SparkMax(canId, motorType);
    this.closedLoopController = m_turretMotor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(TurretConstants.kTurretCurrentLimit);

    config.absoluteEncoder.positionConversionFactor(1.0);
    config.absoluteEncoder.velocityConversionFactor(1.0 / 60.0);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD)
        .iZone(TurretConstants.kTurretIZ)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(0)
        .positionWrappingMaxInput(1);

    m_turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    double maxVelRadPerSec = TurretConstants.kTurretMaxVelocity.in(Units.RadiansPerSecond);
    if (lastAimTarget == null || maxVelRadPerSec <= 0.0) {
      return 0;
    }

    double currentPositionRad = getTurretPosition().in(Units.Radians);
    double distanceRad = Math.abs(lastAimTarget.in(Units.Radians) - currentPositionRad);
    double timeLeftSec = distanceRad / maxVelRadPerSec;
    double timeLeftMs = Math.max(0.0, timeLeftSec) * 1000.0;
    return (int) Math.ceil(timeLeftMs);
  }

  public Angle getTurretPosition() {
    return Units.Rotations.of(m_turretMotor.getAbsoluteEncoder().getPosition());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Turret/PositionRot", getTurretPosition().in(Units.Rotations));
    Logger.recordOutput("Turret/PositionDeg", getTurretPosition().in(Units.Degrees));
    Logger.recordOutput("Turret/Velocity", m_turretMotor.getAbsoluteEncoder().getVelocity());
    Logger.recordOutput("Turret/DesiredOutputRot", lastAimTarget != null ? lastAimTarget.in(Units.Rotations) : 0);
    Logger.recordOutput("Turret/AppliedOutput", m_turretMotor.getAppliedOutput());
    Logger.recordOutput("Turret/BusVoltage", m_turretMotor.getBusVoltage());
  }
}
