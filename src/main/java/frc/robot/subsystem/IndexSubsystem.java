package frc.robot.subsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.IndexConstants;

public class IndexSubsystem extends SubsystemBase {
  private static IndexSubsystem instance;

  private final SparkFlex m_indexMotor;
  private final SparkFlex m_feedMotor;

  public static IndexSubsystem GetInstance() {
    if (instance == null) {
      instance = new IndexSubsystem();
    }
    return instance;
  }

  private IndexSubsystem() {
    m_indexMotor = new SparkFlex(IndexConstants.indexMotorID, MotorType.kBrushless);
    m_feedMotor = new SparkFlex(IndexConstants.feedMotorID, MotorType.kBrushless);
    configureMotors();
  }

  private void configureMotors() {
    SparkFlexConfig indexConfig = new SparkFlexConfig();
    indexConfig.idleMode(IdleMode.kBrake);
    indexConfig.inverted(IndexConstants.indexMotorInverted);

    SparkFlexConfig feedConfig = new SparkFlexConfig();
    feedConfig.idleMode(IdleMode.kBrake);
    feedConfig.inverted(IndexConstants.feedMotorInverted);

    m_indexMotor.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_feedMotor.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runMotors(double speed) {
    m_indexMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
    m_feedMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  public void stopMotors() {
    m_indexMotor.set(0.0);
    m_feedMotor.set(0.0);
  }
}
