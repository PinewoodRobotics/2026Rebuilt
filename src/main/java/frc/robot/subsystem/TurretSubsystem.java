package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
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
import lombok.Setter;

public class TurretSubsystem extends SubsystemBase {
  private static TurretSubsystem instance;

  private SparkFlex m_turretMotor;
  private SparkClosedLoopController closedLoopController;
  private final AbsoluteEncoder absoluteEncoder;

  /** Last commanded turret goal angle (for logging / time estimate). */
  private Angle lastAimTarget;

  @Setter
  private static boolean isGpsAssistEnabled = true;

  public static boolean getIsGpsAssistEnabled() {
    return isGpsAssistEnabled;
  }

  public static TurretSubsystem GetInstance() {
    if (instance == null) {
      instance = new TurretSubsystem(TurretConstants.kTurretCanId, TurretConstants.kTurretMotorType);
    }

    return instance;
  }

  public TurretSubsystem(int canId, MotorType motorType) {
    configureSparkMax(canId, motorType);
    absoluteEncoder = m_turretMotor.getAbsoluteEncoder();
    reset();
  }

  private void configureSparkMax(int canId, MotorType motorType) {
    this.m_turretMotor = new SparkFlex(canId, motorType);
    this.closedLoopController = m_turretMotor.getClosedLoopController();

    SparkFlexConfig config = new SparkFlexConfig();

    config.idleMode(IdleMode.kCoast);
    config.inverted(TurretConstants.kMotorInverted);
    config.openLoopRampRate(0.0);
    config.closedLoopRampRate(0.0);

    config
        .smartCurrentLimit(TurretConstants.kTurretCurrentLimit);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD)
        .iZone(TurretConstants.kTurretIZ)
        .outputRange(-1.0, 1.0)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(0)
        .positionWrappingMaxInput(1);

    config.absoluteEncoder.zeroOffset(TurretConstants.kTurretOffset.in(Units.Rotations)).inverted(true);

    m_turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void reset() {
    // Absolute encoder provides the turret angle reference; nothing to zero here.
    lastAimTarget = getTurretPosition();
  }

  /**
   * Simple position PID (no MAXMotion).
   */
  public void setTurretPosition(Angle position, Voltage feedForward) {
    lastAimTarget = position;
    closedLoopController.setSetpoint(
        lastAimTarget.in(Units.Rotations),
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
    return Units.Rotations.of(absoluteEncoder.getPosition());
  }

  private double wrapToUnitRotations(double rotations) {
    double wrapped = rotations % 1.0;
    if (wrapped < 0.0) {
      wrapped += 1.0;
    }
    return wrapped;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Turret/PositionRot", getTurretPosition().in(Units.Rotations));
    Logger.recordOutput("Turret/PositionDeg", getTurretPosition().in(Units.Degrees));
    Logger.recordOutput("Turret/AbsolutePositionRawRot", absoluteEncoder.getPosition());
    Logger.recordOutput("Turret/Velocity", m_turretMotor.getEncoder().getVelocity());
    Logger.recordOutput("Turret/DesiredOutputRot", lastAimTarget != null ? lastAimTarget.in(Units.Rotations) : 0);
    Logger.recordOutput("Turret/DesiredWrappedOutputRot",
        lastAimTarget != null ? wrapToUnitRotations(lastAimTarget.in(Units.Rotations)) : 0);
    Logger.recordOutput("Turret/AppliedOutput", m_turretMotor.getAppliedOutput());
    Logger.recordOutput("Turret/AppliedOutputWant", m_turretMotor.getOutputCurrent());
    Logger.recordOutput("Turret/BusVoltage", m_turretMotor.getBusVoltage());
    Logger.recordOutput("Turret/TimeTillGoal", getAimTimeLeftMs());
  }
}
