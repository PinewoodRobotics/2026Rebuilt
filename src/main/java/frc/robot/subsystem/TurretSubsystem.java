package frc.robot.subsystem;

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
  private Double lastAimTargetRad = null;

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
        .inverted(TurretConstants.kTurretReversed)
        .smartCurrentLimit(TurretConstants.kTurretCurrentLimit);

    config.absoluteEncoder.positionConversionFactor(1);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD)
        .iZone(TurretConstants.kTurretIZ)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(0)
        .positionWrappingMaxInput(1);

    config.closedLoop.maxMotion
        .cruiseVelocity(TurretConstants.kTurretMaxVelocity.in(Units.RotationsPerSecond))
        .maxAcceleration(TurretConstants.kTurretMaxAcceleration.in(Units.RotationsPerSecondPerSecond));

    m_turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setTurretPosition(Angle position, Voltage feedForward) {
    lastAimTargetRad = position.in(Units.Radians);

    closedLoopController.setSetpoint(
        position.in(Units.Rotations),
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        feedForward.in(Units.Volts),
        ArbFFUnits.kVoltage);
  }

  public int getAimTimeLeftMs() {
    double maxVelRadPerSec = TurretConstants.kTurretMaxVelocity.in(Units.RadiansPerSecond);
    if (lastAimTargetRad == null || maxVelRadPerSec <= 0.0) {
      return 0;
    }

    double currentPositionRad = getTurretPosition().in(Units.Radians);
    double distanceRad = Math.abs(lastAimTargetRad - currentPositionRad);
    double timeLeftSec = distanceRad / maxVelRadPerSec;
    double timeLeftMs = Math.max(0.0, timeLeftSec) * 1000.0;
    return (int) Math.ceil(timeLeftMs);
  }

  public Angle getTurretPosition() {
    return Units.Radians.of(m_turretMotor.getAbsoluteEncoder().getPosition());
  }

  @Override
  public void periodic() {
  }
}
