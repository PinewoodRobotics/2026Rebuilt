package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.constant.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);

  private SparkBase m_turretMotor;
  private SparkClosedLoopController closedLoopController;
  private AbsoluteEncoder absoluteEncoder;
  private RelativeEncoder relativeEncoder;
  private TalonFX m_turnMotor;

  /**
   * Last commanded turret setpoint (radians). Used to compute a live countdown
   * from remaining angular error and {@link TurretConstants#kTurretMaxVelocity}.
   */
  private Double lastAimTargetRad = null;

  private static TurretSubsystem instance;

  public static TurretSubsystem GetInstance() {
    if (instance == null) {
      instance = new TurretSubsystem(TurretConstants.kTurretCanId, TurretConstants.kTurretMotorType);
    }

    return instance;
  }

  public TurretSubsystem(int canId, MotorType motorType) {
    // configureSparkMax(canId, motorType);
    configureSparkMax(canId, motorType);
  }

  @SuppressWarnings("unused")
  private void configureTalonFX(int canId) {
    this.m_turnMotor = new TalonFX(canId);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(
                    TurretConstants.kTurretCurrentLimit)
                .withSupplyCurrentLimit(
                    TurretConstants.kTurretCurrentLimit))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(
                    TurretConstants.kTurretMotorRotationsPerRotation))
        .withSlot0(
            new Slot0Configs()
                .withKP(TurretConstants.kTurretP)
                .withKI(TurretConstants.kTurretI)
                .withKD(TurretConstants.kTurretD))
        .withClosedLoopGeneral(
            new ClosedLoopGeneralConfigs().withContinuousWrap(true))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(TurretConstants.kTurretMaxVelocity)
                .withMotionMagicAcceleration(TurretConstants.kTurretMaxAcceleration)
                .withMotionMagicJerk(0));

    m_turnMotor.getConfigurator().apply(turnConfig);
  }

  @SuppressWarnings("unused")
  private void configureSparkMax(int canId, MotorType motorType) {
    this.m_turretMotor = new SparkMax(canId, motorType);
    this.closedLoopController = m_turretMotor.getClosedLoopController();
    this.absoluteEncoder = m_turretMotor.getAbsoluteEncoder();
    this.relativeEncoder = m_turretMotor.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(TurretConstants.kTurretReversed)
        .smartCurrentLimit(TurretConstants.kTurretCurrentLimit);

    double factor = (2 * Math.PI) / TurretConstants.kTurretMotorRotationsPerRotation;
    config.absoluteEncoder.positionConversionFactor(factor).velocityConversionFactor(factor);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD)
        .iZone(TurretConstants.kTurretIZ)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(0)
        .positionWrappingMaxInput(1);

    config.absoluteEncoder.positionConversionFactor(1);

    // These limits are enforced when using kMAXMotionPositionControl.
    config.closedLoop.maxMotion
        // With velocity in rad/min, express cruise as rad/min and accel as (rad/min)/s.
        .cruiseVelocity(TurretConstants.kTurretMaxVelocity.in(Units.RadiansPerSecond) * 60.0)
        .maxAcceleration(TurretConstants.kTurretMaxAcceleration.in(Units.RadiansPerSecondPerSecond) * 60.0);

    m_turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * SPARK Flex equivalent of {@link #configureSparkMax(int, MotorType)} using
   * REVLib 2026's Spark "configure" API.
   */
  private void configureSparkFlex(int canId, MotorType motorType) {
    this.m_turretMotor = new SparkFlex(canId, motorType);
    this.closedLoopController = m_turretMotor.getClosedLoopController();
    this.relativeEncoder = m_turretMotor.getEncoder();

    SparkFlexConfig config = new SparkFlexConfig();
    config
        .inverted(TurretConstants.kTurretReversed)
        .smartCurrentLimit(TurretConstants.kTurretCurrentLimit);

    // Keep units consistent across the whole turret stack:
    // - Position: radians
    // - Velocity: rad/min (Spark reports RPM by default)
    //
    // This matches REV's MAXMotion parameter conventions (RPM and RPM/s by
    // default).
    double factor = (2 * Math.PI) / TurretConstants.kTurretMotorRotationsPerRotation;
    config.encoder.positionConversionFactor(factor).velocityConversionFactor(factor);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD)
        .iZone(TurretConstants.kTurretIZ)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(-Math.PI)
        .positionWrappingMaxInput(Math.PI);

    // These limits are enforced when using kMAXMotionPositionControl.
    config.closedLoop.maxMotion
        // With velocity in rad/min, express cruise as rad/min and accel as (rad/min)/s.
        .cruiseVelocity(TurretConstants.kTurretMaxVelocity.in(Units.RadiansPerSecond) * 60.0)
        .maxAcceleration(TurretConstants.kTurretMaxAcceleration.in(Units.RadiansPerSecondPerSecond) * 60.0);

    m_turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setTurretPosition(Angle position, Voltage feedForward) {
    lastAimTargetRad = position.in(Units.Radians);

    if (closedLoopController != null) {
      closedLoopController.setSetpoint(
          position.in(Units.Revolutions),
          ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0,
          feedForward.in(Units.Volts),
          ArbFFUnits.kVoltage);
    } else if (m_turnMotor != null) {
      m_turnMotor.setControl(positionRequest.withPosition(position).withFeedForward(feedForward));
    }
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
    // Use the same sensor the controller uses (primary encoder).
    if (m_turnMotor != null) {
      return m_turnMotor.getPosition().getValue();
    } else {
      return Units.Radians.of(relativeEncoder.getPosition());
    }
  }

  @Override
  public void periodic() {
    // Log turret position in both radians and degrees
    Logger.recordOutput("Turret/PositionRadians", getTurretPosition().in(Units.Radians));
    Logger.recordOutput("Turret/PositionDegrees", getTurretPosition().in(Units.Degrees));
    Logger.recordOutput("Turret/AimTimeLeftMs", getAimTimeLeftMs());

    // Attempt to log velocity if available, otherwise log 0 or a placeholder
    double velocityRadPerSec = 0.0;
    double velocityDegPerSec = 0.0;
    if (absoluteEncoder != null) {
      // With conversion factors above, absolute encoder velocity is rad/min.
      velocityRadPerSec = absoluteEncoder.getVelocity() / 60.0;
      velocityDegPerSec = Math.toDegrees(velocityRadPerSec);
    } else if (relativeEncoder != null) {
      // With conversion factors above, relative encoder velocity is rad/min.
      velocityRadPerSec = relativeEncoder.getVelocity() / 60.0;
      velocityDegPerSec = Math.toDegrees(velocityRadPerSec);
    }
    Logger.recordOutput("Turret/VelocityRadiansPerSec", velocityRadPerSec);
    Logger.recordOutput("Turret/VelocityDegreesPerSec", velocityDegPerSec);

    // Log raw encoder readings for diagnostic purposes
    if (absoluteEncoder != null) {
      Logger.recordOutput("Turret/RawAbsoluteEncoderPosition", absoluteEncoder.getPosition());
      Logger.recordOutput("Turret/RawAbsoluteEncoderVelocity", absoluteEncoder.getVelocity());
    }
    if (m_turretMotor != null) {
      Logger.recordOutput("Turret/AppliedOutput", m_turretMotor.getAppliedOutput());
      Logger.recordOutput("Turret/BusVoltage", m_turretMotor.getBusVoltage());
      Logger.recordOutput("Turret/OutputCurrent", m_turretMotor.getOutputCurrent());
    }
  }
}
