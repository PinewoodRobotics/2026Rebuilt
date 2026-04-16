package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.ClimberConstants;
import frc.robot.util.LocalMath;

public class ClimberSubsystem extends SubsystemBase {
  private static final double kNominalVoltage = 12.0;

  private static ClimberSubsystem self;

  public enum ControlType {
    HEIGHT,
    VELOCITY,
  }

  private final SparkFlex m_climbMotor;
  private final PIDController m_heightPid;
  private final PIDController m_velocityPid;
  private final ElevatorFeedforward m_feedforward;

  private ControlType m_controlType = ControlType.HEIGHT;
  private Distance m_setpoint = ClimberConstants.kStartingHeight;
  private Distance m_rampedSetpoint = ClimberConstants.kStartingHeight;
  private LinearVelocity m_velocitySetpoint = MetersPerSecond.of(0.0);

  public static ClimberSubsystem GetInstance() {
    if (self == null) {
      self = new ClimberSubsystem();
    }

    return self;
  }

  public ClimberSubsystem() {
    m_climbMotor = new SparkFlex(ClimberConstants.kClimberMotorID, ClimberConstants.kMotorType);
    m_heightPid = new PIDController(
        ClimberConstants.kP,
        ClimberConstants.kI,
        ClimberConstants.kD);
    m_velocityPid = new PIDController(
        ClimberConstants.kVelocityP,
        ClimberConstants.kVelocityI,
        ClimberConstants.kVelocityD);
    m_feedforward = new ElevatorFeedforward(
        ClimberConstants.kS,
        ClimberConstants.kG,
        ClimberConstants.kV,
        ClimberConstants.kA);

    m_heightPid.setTolerance(ClimberConstants.kTolerance);
    m_heightPid.setIZone(ClimberConstants.kIZone);

    configureMotor();
  }

  private void configureMotor() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(ClimberConstants.kMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.kLiftCurrentLimit)
        .openLoopRampRate(ClimberConstants.kOpenLoopRampSeconds)
        .closedLoopRampRate(ClimberConstants.kClosedLoopRampSeconds);
    config.encoder
        .positionConversionFactor(ClimberConstants.kGearHeightRatio)
        .velocityConversionFactor(ClimberConstants.kGearHeightRatio / 60.0);

    m_climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_climbMotor.getEncoder().setPosition(ClimberConstants.kStartingHeight.in(Meters));
    m_rampedSetpoint = ClimberConstants.kStartingHeight;
  }

  private Distance clampHeight(Distance height) {
    return Meters.of(MathUtil.clamp(
        height.in(Meters),
        ClimberConstants.kMinHeight.in(Meters),
        ClimberConstants.kMaxHeight.in(Meters)));
  }

  public void setHeight(Distance height) {
    m_controlType = ControlType.HEIGHT;
    m_setpoint = clampHeight(height);
  }

  public void setVelocity(LinearVelocity velocity) {
    m_controlType = ControlType.VELOCITY;
    m_velocitySetpoint = velocity;
  }

  public Distance getHeight() {
    return Meters.of(m_climbMotor.getEncoder().getPosition());
  }

  public Distance getAverageHeight() {
    return getHeight();
  }

  public double getHeightVelocityMetersPerSecond() {
    return m_climbMotor.getEncoder().getVelocity();
  }

  public LinearVelocity getHeightVelocity() {
    return MetersPerSecond.of(getHeightVelocityMetersPerSecond());
  }

  public double getAppliedVoltage() {
    return m_climbMotor.getAppliedOutput() * m_climbMotor.getBusVoltage();
  }

  public double getOutputCurrentAmps() {
    return m_climbMotor.getOutputCurrent();
  }

  public boolean isCalibrationVelocitySettled() {
    return Math.abs(getHeightVelocityMetersPerSecond()) <= ClimberConstants.kCalibrationVelocityToleranceMetersPerSecond
        || m_climbMotor.getOutputCurrent() >= ClimberConstants.kCalibrationVoltageTolerance.in(Units.Amps);
  }

  public void zeroHeightEncoder() {
    m_climbMotor.getEncoder().setPosition(ClimberConstants.kMinHeight.in(Meters));
  }

  private double calculateFeedForwardValue() {
    return m_feedforward.calculate(0.0);
  }

  private double calculateHeightControlVoltage(Distance setpoint) {
    double motorPowerPid = m_heightPid.calculate(getHeight().in(Meters), setpoint.in(Meters));
    return MathUtil.clamp(motorPowerPid + calculateFeedForwardValue(), -1.0, 1.0) * kNominalVoltage;
  }

  private double calculateVelocityControlVoltage(LinearVelocity velocitySetpoint) {
    double targetVelocityMetersPerSecond = velocitySetpoint.in(MetersPerSecond);
    double currentVelocityMetersPerSecond = getHeightVelocityMetersPerSecond();
    double velocityPid = m_velocityPid.calculate(currentVelocityMetersPerSecond, targetVelocityMetersPerSecond);
    double velocityFeedforward = m_feedforward.calculate(targetVelocityMetersPerSecond);
    return MathUtil.clamp(velocityPid + velocityFeedforward, -kNominalVoltage, kNominalVoltage);
  }

  private double clampHeightControlVoltage(double requestedVoltage) {
    double clampedVoltage = MathUtil.clamp(requestedVoltage, -kNominalVoltage, kNominalVoltage);
    double currentHeightMeters = getHeight().in(Meters);

    if (currentHeightMeters >= ClimberConstants.kMaxHeight.in(Meters) && clampedVoltage > 0.0) {
      return 0.0;
    }
    if (currentHeightMeters <= ClimberConstants.kMinHeight.in(Meters) && clampedVoltage < 0.0) {
      return 0.0;
    }

    return clampedVoltage;
  }

  public boolean atTarget() {
    return Math.abs(getHeight().minus(m_setpoint).in(Meters)) < ClimberConstants.kTolerance;
  }

  public void stopMotors() {
    stopHeightMotor();
  }

  public void stopHeightMotor() {
    Distance currentHeight = getHeight();
    m_controlType = ControlType.HEIGHT;
    m_setpoint = currentHeight;
    m_rampedSetpoint = currentHeight;
    m_velocitySetpoint = MetersPerSecond.of(0.0);
    m_heightPid.reset();
    m_velocityPid.reset();
  }

  private Distance rampSetpoint(Distance setpoint) {
    return Meters.of(LocalMath.rampSetpoint(
        setpoint.in(Meters),
        m_rampedSetpoint.in(Meters),
        ClimberConstants.kMaxSetpointRamp));
  }

  private Distance calculateTemporarySetpoint(Distance setpoint) {
    if (ClimberConstants.kSetpointRamping) {
      return rampSetpoint(setpoint);
    }

    return setpoint;
  }

  public void resetIAccum() {
    m_heightPid.reset();
    m_velocityPid.reset();
  }

  @Override
  public void periodic() {
    Distance currentHeight = getHeight();
    double requestedVoltage;

    switch (m_controlType) {
      case VELOCITY:
        m_rampedSetpoint = currentHeight;
        requestedVoltage = calculateVelocityControlVoltage(m_velocitySetpoint);
        break;
      case HEIGHT:
      default:
        m_rampedSetpoint = calculateTemporarySetpoint(m_setpoint);
        requestedVoltage = calculateHeightControlVoltage(m_rampedSetpoint);
        break;
    }

    double appliedVoltage = m_controlType == ControlType.VELOCITY
        ? requestedVoltage
        : clampHeightControlVoltage(requestedVoltage);

    m_climbMotor.setVoltage(appliedVoltage);

    Logger.recordOutput("Climber/CurrentHeightMeters", currentHeight.in(Meters));
    Logger.recordOutput("Climber/ControlType", m_controlType.toString());
    Logger.recordOutput("Climber/TargetHeightMeters", m_setpoint.in(Meters));
    Logger.recordOutput("Climber/RampedSetpointMeters", m_rampedSetpoint.in(Meters));
    Logger.recordOutput("Climber/VelocitySetpointMetersPerSecond", m_velocitySetpoint.in(MetersPerSecond));
    Logger.recordOutput("Climber/AtTarget", atTarget());
    Logger.recordOutput("Climber/RequestedVoltage", requestedVoltage);
    Logger.recordOutput("Climber/AppliedVoltageCommand", appliedVoltage);
    Logger.recordOutput("Climber/VelocityMetersPerSecond", getHeightVelocityMetersPerSecond());
    Logger.recordOutput("Climber/AppliedVoltage", getAppliedVoltage());
    Logger.recordOutput("Climber/OutputCurrentAmps", getOutputCurrentAmps());
    Logger.recordOutput("Climber/CalibrationVelocitySettled", isCalibrationVelocitySettled());
    Logger.recordOutput("Climber/AppliedOutput", m_climbMotor.getAppliedOutput());
    Logger.recordOutput("Climber/OutputCurrent", getOutputCurrentAmps());
  }
}
