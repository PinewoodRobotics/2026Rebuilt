package frc.robot.subsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private static ClimberSubsystem instance;

  private SparkMax climberMotor;
  private final SparkClosedLoopController climberClosedLoopController;

  public static ClimberSubsystem GetInstance() {
    if (instance == null) {
      instance = new ClimberSubsystem();
    }

    return instance;
  }

  private ClimberSubsystem() {
    this.climberMotor = new SparkMax(ClimberConstants.climberMotorID, ClimberConstants.climberMotorType);
    this.climberClosedLoopController = climberMotor.getClosedLoopController();
    configureSparkMax();
  }

  private void configureSparkMax() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(ClimberConstants.climberMotorInverted);
    climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private Angle getAngleFromDistance(Distance distance) {
    return Angle.ofRelativeUnits(
        distance.in(Units.Meters) / ClimberConstants.gearDiameter.in(Units.Meters) * 2.0 * Math.PI, Units.Rotations);
  }

  public void setPosition(Distance distance) {
    Angle angle = getAngleFromDistance(distance);
    climberClosedLoopController.setSetpoint(angle.in(Units.Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot0,
        0.0);
  }
}