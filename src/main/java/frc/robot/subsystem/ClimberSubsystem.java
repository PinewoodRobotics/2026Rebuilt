package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.ClimberConstants;
import frc.robot.util.LocalMath;

public class ClimberSubsystem extends SubsystemBase {

  private static ClimberSubsystem self;

  private SparkMax m_climbMotor;
  private SparkMax m_wristMotor;
  private PIDController m_pid;
  private ElevatorFeedforward m_feedforward;
  private boolean m_isHeightVoltageControl = false;
  private double m_heightVoltagePercent = 0.0;
  private double m_wristVoltagePercent = 0.0;

  private Distance m_setpoint = ClimberConstants.kStartingHeight;
  private Distance m_currentPosition = ClimberConstants.kStartingHeight;

  public static ClimberSubsystem GetInstance() {
    if (self == null) {
      self = new ClimberSubsystem();
    }

    return self;
  }

  public ClimberSubsystem() {
    m_climbMotor = new SparkMax(ClimberConstants.kLeftMotorID, ClimberConstants.kMotorType);
    m_wristMotor = new SparkMax(ClimberConstants.kWristMotorID, ClimberConstants.kMotorType);

    m_pid = new PIDController(
        ClimberConstants.kP,
        ClimberConstants.kI,
        ClimberConstants.kD);
    m_pid.setTolerance(ClimberConstants.kTolerance);
    m_feedforward = new ElevatorFeedforward(ClimberConstants.kS, ClimberConstants.kG, ClimberConstants.kV,
        ClimberConstants.kA);

    configureMotors();
  }

  private void configureMotors() {
    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig.inverted(ClimberConstants.kLeftMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.kLiftCurrentLimit);
    leftMotorConfig.encoder
        .positionConversionFactor(ClimberConstants.kGearHeightRatio)
        .velocityConversionFactor(ClimberConstants.kGearHeightRatio / 60.0);

    m_climbMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_climbMotor.getEncoder().setPosition(ClimberConstants.kStartingHeight.in(Meters));

    SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
    wristMotorConfig.inverted(ClimberConstants.kWristMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ClimberConstants.kWristCurrentLimit);
    wristMotorConfig.encoder
        .positionConversionFactor(ClimberConstants.kWristGearRatio)
        .velocityConversionFactor(ClimberConstants.kWristGearRatio / 60.0);
    m_wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_wristMotor.stopMotor();

    m_pid.setIZone(ClimberConstants.kIZone);
  }

  public void setHeight(Distance height) {
    m_isHeightVoltageControl = false;
    if (height.gt(ClimberConstants.kMaxHeight)) {
      System.out.println("WARNING: tried to exceed elevator max height: " + height.in(Meters));
      height = ClimberConstants.kMaxHeight;
    } else if (height.lt(ClimberConstants.kMinHeight)) {
      System.out.println("WARNING: tried to exceed elevator min height: " + height.in(Meters));
      height = ClimberConstants.kMinHeight;
    }
    m_setpoint = height;
  }

  public void setHeightPercent(double heightPercent) {
    double clampedPercent = MathUtil.clamp(heightPercent, 0.0, 1.0);
    double targetHeightMeters = MathUtil.interpolate(
        ClimberConstants.kMinHeight.in(Meters),
        ClimberConstants.kMaxHeight.in(Meters),
        clampedPercent);
    setHeight(Meters.of(targetHeightMeters));
  }

  public void setHeightVoltagePercent(double heightVoltagePercent) {
    m_isHeightVoltageControl = true;
    m_heightVoltagePercent = MathUtil.clamp(heightVoltagePercent, 0.0, 1.0);
  }

  public Distance getAverageHeight() {
    double height = m_climbMotor.getEncoder().getPosition();
    return Distance.ofRelativeUnits(height, Meters);
  }

  private double calculateSpeed(Distance setpoint) {
    double motorPowerPid = m_pid.calculate(getAverageHeight().in(Meters), setpoint.in(Meters));
    double ff = calculateFeedForwardValue(m_feedforward);
    return MathUtil.clamp(motorPowerPid + ff, -1, 1);
  }

  public boolean atTarget() {
    return Math.abs(getAverageHeight().minus(m_setpoint).in(Meters)) < ClimberConstants.kTolerance;
  }

  public void stopMotors() {
    stopHeightMotor();
    stopWristMotor();
  }

  public void stopHeightMotor() {
    m_heightVoltagePercent = 0.0;
    m_climbMotor.stopMotor();
  }

  public double getWristPositionRotations() {
    if (m_wristMotor == null) {
      return 0.0;
    }

    return m_wristMotor.getEncoder().getPosition();
  }

  public void setWristEncoderPosition(double rotations) {
    if (m_wristMotor == null) {
      return;
    }

    m_wristMotor.getEncoder().setPosition(rotations);
  }

  public void setWristVoltagePercent(double wristVoltagePercent) {
    if (m_wristMotor == null) {
      return;
    }

    m_wristVoltagePercent = MathUtil.clamp(wristVoltagePercent, 0.0, 1.0);
  }

  public void zeroWristEncoder() {
    setWristEncoderPosition(0.0);
  }

  public void stopWristMotor() {
    if (m_wristMotor != null) {
      m_wristVoltagePercent = 0.0;
      m_wristMotor.stopMotor();
    }
  }

  public void stopWrist() {
    stopWristMotor();
  }

  private double calculateFeedForwardValue(ElevatorFeedforward feedforward) {
    double currentVelocity = m_climbMotor.getEncoder().getVelocity();
    return feedforward.calculate(currentVelocity);
  }

  private Distance rampSetpoint(Distance set) {
    return Distance.ofRelativeUnits(
        LocalMath.rampSetpoint(set.in(Meters), m_currentPosition.in(Meters), ClimberConstants.kMaxSetpointRamp),
        Meters);
  }

  private Distance calculateTemporarySetpoint(Distance set) {
    if (ClimberConstants.kSetpointRamping) {
      set = rampSetpoint(set);
    }
    return set;
  }

  public void resetIAccum() {
    m_pid.reset();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/Setpoint", m_setpoint.in(Meters));
    Logger.recordOutput("Climber/CurrentPosition", m_currentPosition.in(Meters));
    Logger.recordOutput("Climber/AtTarget", atTarget());
    Logger.recordOutput("Climber/LeftMotor", m_climbMotor.getEncoder().getPosition());
    Logger.recordOutput("Climber/HeightVoltageControl", m_isHeightVoltageControl);
    Logger.recordOutput("Climber/HeightVoltagePercent", m_heightVoltagePercent);
    Logger.recordOutput("Climber/WristConfigured", m_wristMotor != null);
    Logger.recordOutput("Climber/WristVoltagePercent", m_wristVoltagePercent);
    if (m_wristMotor != null) {
      Logger.recordOutput("Climber/WristPosition", getWristPositionRotations());
    }

    m_currentPosition = calculateTemporarySetpoint(m_setpoint);

    if (m_isHeightVoltageControl) {
      m_climbMotor.setVoltage(m_heightVoltagePercent * 12.0);
    } else {
      double speed = calculateSpeed(m_currentPosition);
      m_climbMotor.setVoltage(speed * 12.0);
    }
    if (m_wristMotor != null) {
      m_wristMotor.setVoltage(m_wristVoltagePercent * 12.0);
    }
  }
}
