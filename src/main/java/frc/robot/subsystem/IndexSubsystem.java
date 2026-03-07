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

  public static IndexSubsystem GetInstance() {
    if (instance == null) {
      instance = new IndexSubsystem();
    }
    return instance;
  }

  private IndexSubsystem() {
    m_indexMotor = new SparkFlex(IndexConstants.kIndexMotorID, MotorType.kBrushless);
    configureMotor();
  }

  private void configureMotor() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(true);

    m_indexMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runMotor(double speed) {
    m_indexMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  public void runMotor() {
    runMotor(IndexConstants.kIndexMotorSpeed);
  }

  public void stopMotor() {
    m_indexMotor.set(0.0);
  }
}
