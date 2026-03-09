package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.IntakeConstants;
import frc.robot.constant.IntakeConstants.WristRaiseLocation;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem instance;

  private final SparkMax m_intakeIntakerMotor;
  private final SparkMax m_intakeWristMotor;

  private Rotation2d m_wristSetpoint = IntakeConstants.WristRaiseLocation.BOTTOM.position;

  public static IntakeSubsystem GetInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem(IntakeConstants.intakeIntakerMotorID, IntakeConstants.intakeWristMotorID);
    }

    return instance;
  }

  private IntakeSubsystem(int intakeMotorID, int wristMotorID) {
    m_intakeIntakerMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
    m_intakeWristMotor = new SparkMax(wristMotorID, MotorType.kBrushless);
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
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    wristConfig.idleMode(IdleMode.kBrake);
    wristConfig.closedLoop.pid(
        IntakeConstants.intakeWristP,
        IntakeConstants.intakeWristI,
        IntakeConstants.intakeWristD)
        .iZone(IntakeConstants.intakeWristIZone);

    wristConfig.absoluteEncoder.zeroOffset(IntakeConstants.intakeWristOffset.getRotations());

    m_intakeWristMotor.configure(wristConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private double calculateFeedForward() {
    return IntakeConstants.intakeWristFeedForwardK
        * Math.cos(getWristPosition().getRadians());
  }

  public Rotation2d getWristPosition() {
    return Rotation2d.fromRotations(m_intakeWristMotor.getAbsoluteEncoder().getPosition());
  }

  private void setWristPosition(Rotation2d position) {
    m_wristSetpoint = position;
  }

  public void setWristPosition(WristRaiseLocation location) {
    setWristPosition(location.position);
  }

  public void runIntakeMotor(double speed) {
    m_intakeIntakerMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  public void stopIntakeMotor() {
    m_intakeIntakerMotor.set(0.0);
  }

  @Override
  public void periodic() {
    if (m_wristSetpoint != null) {
      m_intakeWristMotor.getClosedLoopController().setSetpoint(
          m_wristSetpoint.getRotations(),
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          calculateFeedForward());
    }

    Logger.recordOutput("IntakeSubsystem/WristPosition", getWristPosition().getRotations());
    Logger.recordOutput("IntakeSubsystem/WristSetpoint", m_wristSetpoint.getRotations());
    Logger.recordOutput("IntakeSubsystem/FeedForward", calculateFeedForward());
  }
}
