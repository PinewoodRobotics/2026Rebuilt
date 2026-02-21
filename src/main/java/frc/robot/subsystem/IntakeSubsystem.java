package frc.robot.subsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem instance;

  private final SparkMax m_intakeIntakerMotor;
  private final SparkMax m_intakeWristMotor;

  private Rotation2d m_wristSetpoint = IntakeConstants.intakeWristStowedAngle;

  public static IntakeSubsystem GetInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  private IntakeSubsystem() {
    m_intakeIntakerMotor = new SparkMax(IntakeConstants.intakeIntakerMotorID, MotorType.kBrushless);
    m_intakeWristMotor = new SparkMax(IntakeConstants.intakeWristMotorID, MotorType.kBrushless);
    configureIntaker();
    configureWrist();
  }

  private void configureIntaker() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(IntakeConstants.intakeIntakerInverted);

    m_intakeIntakerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureWrist() {
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig.smartCurrentLimit(IntakeConstants.intakeWristCurrentLimit);
    wristConfig.inverted(IntakeConstants.intakeWristInverted);
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    wristConfig.encoder.positionConversionFactor(IntakeConstants.intakeWristGearingRatio);
    wristConfig.idleMode(IdleMode.kBrake);
    wristConfig.closedLoop.pid(
        IntakeConstants.intakeWristP,
        IntakeConstants.intakeWristI,
        IntakeConstants.intakeWristD)
        .iZone(IntakeConstants.intakeWristIZone);

    wristConfig.absoluteEncoder.inverted(true).zeroOffset(IntakeConstants.intakeWristOffset.getRotations());

    m_intakeWristMotor.configure(wristConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    calibrateWrist();
  }

  private double calculateFeedForward() {
    return IntakeConstants.intakeWristFeedForwardK
        * Math.cos(getWristPosition().getRadians() - IntakeConstants.intakeWristFFOffset.getRadians());
  }

  public void stopWrist() {
    setWristPosition(getWristPosition());
  }

  public Rotation2d getWristPosition() {
    return Rotation2d.fromRotations(m_intakeWristMotor.getEncoder().getPosition());
  }

  public void setWristPosition(Rotation2d position) {
    m_wristSetpoint = position;
    System.out.println("moving the wrist!");
  }

  public Rotation2d getSetpoint() {
    return m_wristSetpoint;
  }

  public boolean atSetpoint() {
    return Math.abs(getWristPosition().minus(m_wristSetpoint).getRotations()) < IntakeConstants.kTolerance
        .getRotations();
  }

  public void calibrateWrist() {
    m_intakeWristMotor.getEncoder()
        .setPosition(plusMinusHalf(m_intakeWristMotor.getAbsoluteEncoder().getPosition()));
  }

  private static double plusMinusHalf(double in) {
    while (in > 0.5) {
      in -= 1;
    }
    while (in < -0.5) {
      in += 1;
    }
    return in;
  }

  public void runMotor(double speed) {
    m_intakeIntakerMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  public void stopMotor() {
    m_intakeIntakerMotor.set(0.0);
  }

  @Override
  public void periodic() {
    m_intakeWristMotor.getClosedLoopController().setSetpoint(
        m_wristSetpoint.getRotations(),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        calculateFeedForward());
    System.out.printf(
        "current pos is %.3f rot, trying to go to %.3f rot%n",
        getWristPosition().getRotations(),
        getSetpoint().getRotations());

  }
}
