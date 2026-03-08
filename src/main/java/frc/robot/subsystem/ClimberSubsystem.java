package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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

  // LEFT MOTOR IS THE LEADER
  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;
  private PIDController m_pid;
  private ElevatorFeedforward m_feedforward;

  private Distance m_setpoint = ClimberConstants.kStartingHeight;
  private Distance m_currentPosition = ClimberConstants.kStartingHeight;

  public static ClimberSubsystem GetInstance() {
    if (self == null) {
      self = new ClimberSubsystem();
    }

    return self;
  }

  public ClimberSubsystem() {
    m_leftMotor = new SparkMax(ClimberConstants.kLeftMotorID, MotorType.kBrushless);
    m_rightMotor = new SparkMax(ClimberConstants.kRightMotorID, MotorType.kBrushless);

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
        .smartCurrentLimit(30).encoder.positionConversionFactor(ClimberConstants.kGearHeightRatio);

    m_leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftMotor.getEncoder().setPosition(ClimberConstants.kStartingHeight.in(Meters));

    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig.inverted(ClimberConstants.kRightMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30).encoder.positionConversionFactor(ClimberConstants.kGearHeightRatio);

    m_rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.getEncoder().setPosition(ClimberConstants.kStartingHeight.in(Meters));

    m_pid.setIZone(ClimberConstants.kIZone);
  }

  public void setHeight(Distance height) {
    if (height.gt(ClimberConstants.kMaxHeight)) {
      System.out.println("WARNING: tried to exceed elevator max height: " + height.in(Meters));
      height = ClimberConstants.kMaxHeight;
    } else if (height.lt(ClimberConstants.kMinHeight)) {
      System.out.println("WARNING: tried to exceed elevator min height: " + height.in(Meters));
      height = ClimberConstants.kMinHeight;
    }
    m_setpoint = height;
  }

  public Distance getAverageHeight() {
    double height = (m_leftMotor.getEncoder().getPosition() + m_rightMotor.getEncoder().getPosition()) / 2.0;
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
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
  }

  private double calculateFeedForwardValue(ElevatorFeedforward feedforward) {
    double currentVelocity = m_leftMotor.getEncoder().getVelocity();
    return feedforward.calculate(currentVelocity);
  }

  private Distance rampSetpoint(Distance set) {
    return Distance.ofRelativeUnits(
        LocalMath.rampSetpoint(set.in(Meters), m_currentPosition.in(Meters), ClimberConstants.kMaxSetpointRamp),
        Meters);
  }

  private Distance calculateTemporarySetpoint(Distance set) {
    // set = smoothRestingHeight(set);
    set = rampSetpoint(set);
    return set;
  }

  public void resetIAccum() {
    m_leftMotor.getClosedLoopController().setIAccum(0);
    m_rightMotor.getClosedLoopController().setIAccum(0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/Setpoint", m_setpoint.in(Meters));
    Logger.recordOutput("Climber/CurrentPosition", m_currentPosition.in(Meters));
    Logger.recordOutput("Climber/AtTarget", atTarget());
    Logger.recordOutput("Climber/LeftMotor", m_leftMotor.getEncoder().getPosition());
    Logger.recordOutput("Climber/RightMotor", m_rightMotor.getEncoder().getPosition());

    m_currentPosition = calculateTemporarySetpoint(m_setpoint);

    double speed = calculateSpeed(m_currentPosition);

    m_leftMotor.setVoltage(speed * 12);
    m_rightMotor.setVoltage(speed * 12);
  }
}
