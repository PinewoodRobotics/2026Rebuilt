package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.IntakeConstants;
import frc.robot.constant.IntakeConstants.WristRaiseLocation;
import frc.robot.util.SharedDoubleTopic;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem instance;

  private final SparkFlex m_intakeIntakerMotor;
  private final SparkFlex m_intakeWristMotor;
  private final SparkFlex m_intakeWristFollowerMotor;

  private final Timer m_wristDropTimer = new Timer();
  private boolean m_wristDropDone = false;

  private Rotation2d m_wristSetpoint = IntakeConstants.WristRaiseLocation.BOTTOM.position;
  private final SharedDoubleTopic m_intakeSpeedTopic = new SharedDoubleTopic(IntakeConstants.kIntakeSpeedTopic, -1.0);

  public static IntakeSubsystem GetInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem(IntakeConstants.intakeIntakerMotorID, IntakeConstants.intakeWristMotorID);
    }

    return instance;
  }

  private IntakeSubsystem(int intakeMotorID, int wristMotorID) {
    m_intakeIntakerMotor = new SparkFlex(intakeMotorID, MotorType.kBrushless);
    m_intakeWristMotor = new SparkFlex(wristMotorID, MotorType.kBrushless);
    m_intakeWristFollowerMotor = new SparkFlex(IntakeConstants.intakeWristFollowerMotorID, MotorType.kBrushless);
    configureIntaker();
    configureWrist();
    configureWristFollower();
  }

  private void configureIntaker() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(IntakeConstants.intakeIntakerInverted);

    m_intakeIntakerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureWrist() {
    SparkFlexConfig wristConfig = new SparkFlexConfig();
    wristConfig.smartCurrentLimit(IntakeConstants.intakeWristCurrentLimit);
    wristConfig.inverted(IntakeConstants.intakeWristInverted);
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    wristConfig.absoluteEncoder.inverted(IntakeConstants.intakeWristEncoderInverted);
    wristConfig.idleMode(IdleMode.kCoast);
    // wristConfig.closedLoop.pid(
    //     IntakeConstants.intakeWristP,
    //     IntakeConstants.intakeWristI,
    //     IntakeConstants.intakeWristD)
    //     .iZone(IntakeConstants.intakeWristIZone)
    //     .positionWrappingEnabled(true)
    //     .positionWrappingMinInput(0)
    //     .positionWrappingMinInput(1);

    wristConfig.absoluteEncoder.zeroOffset(IntakeConstants.intakeWristOffset.getRotations());

    m_intakeWristMotor.configure(wristConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private void configureWristFollower() {
    SparkFlexConfig followerConfig = new SparkFlexConfig();
    followerConfig.smartCurrentLimit(IntakeConstants.intakeWristCurrentLimit);
    followerConfig.idleMode(IdleMode.kCoast);
    followerConfig.follow(m_intakeWristMotor, true);

    m_intakeWristFollowerMotor.configure(followerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private void updateWristDrop() {
    if (m_wristDropDone || !DriverStation.isEnabled()) {
      return;
    }

    m_wristDropTimer.start();
    double position = getWristPosition().getRotations();
    boolean released = position >= IntakeConstants.kWristDropReleaseWindowMin
        && position <= IntakeConstants.kWristDropReleasePosition;
    if (released || m_wristDropTimer.hasElapsed(IntakeConstants.kWristDropTimeoutS)) {
      m_intakeWristMotor.stopMotor();
      m_wristDropDone = true;
      return;
    }

    m_intakeWristMotor.set(IntakeConstants.kWristDropSpeed);
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

  /**
   * Intake duty cycle: the dashboard override when one is in range, otherwise the
   * compiled constant.
   */
  public double getIntakeSpeed() {
    double requested = m_intakeSpeedTopic.getRequest();
    boolean inRange = Double.isFinite(requested) && requested >= 0.0 && requested <= 1.0;
    return inRange ? requested : IntakeConstants.intakeMotorSpeed;
  }

  public void runIntakeMotor(double speed) {
    m_intakeIntakerMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  public void stopIntakeMotor() {
    m_intakeIntakerMotor.set(0.0);
  }

  public boolean wristAtSetpoint() {
    // return Math.abs(getWristPosition().minus(m_wristSetpoint).getRadians()) < IntakeConstants.kTolerance.getRadians();
    return true;
  }

  @Override
  public void periodic() {
    // if (m_wristSetpoint != null) {
    //   m_intakeWristMotor.getClosedLoopController().setSetpoint(
    //       m_wristSetpoint.getRotations(),
    //       ControlType.kPosition,
    //       ClosedLoopSlot.kSlot0,
    //       calculateFeedForward());
    // }
    updateWristDrop();

    double intakeSpeed = getIntakeSpeed();
    m_intakeSpeedTopic.setState(intakeSpeed);
    Logger.recordOutput("IntakeSubsystem/IntakeSpeed", intakeSpeed);
    Logger.recordOutput("IntakeSubsystem/WristPosition", getWristPosition().getRotations());
    Logger.recordOutput("IntakeSubsystem/WristSetpoint", m_wristSetpoint.getRotations());
    Logger.recordOutput("IntakeSubsystem/FeedForward", calculateFeedForward());
    Logger.recordOutput("IntakeSubsystem/WristDropDone", m_wristDropDone);
  }
}
