package frc.robot.subsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constant.TurretConstants;

public class TurretSubsystem {
    private final SparkMax m_turretMotor;
    private final SparkClosedLoopController closedLoopController;
    private final AbsoluteEncoder absoluteEncoder;

    public TurretSubsystem(int canId, MotorType motorType) {
        this.m_turretMotor = new SparkMax(canId, motorType);
        this.closedLoopController = m_turretMotor.getClosedLoopController();
        this.absoluteEncoder = m_turretMotor.getAbsoluteEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(TurretConstants.kTurretReversed)
                .smartCurrentLimit(TurretConstants.kTurretCurrentLimit);

        double factor = (2 * Math.PI) / TurretConstants.kTurretMotorRotationsPerRotation;
        config.encoder.positionConversionFactor(factor / 60).velocityConversionFactor(factor / 60);
        config.absoluteEncoder.positionConversionFactor(factor / 60).velocityConversionFactor(factor / 60);

        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD)
                .iZone(TurretConstants.kTurretIZ)
                .positionWrappingEnabled(true)
                .positionWrappingMinInput(-Math.PI)
                .positionWrappingMaxInput(Math.PI);

        m_turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTurretPosition(Angle position) {
        closedLoopController.setReference(position.in(Units.Radians), ControlType.kPosition);
    }

    public Angle getTurretPosition() {
        return Angle.ofRelativeUnits(absoluteEncoder.getPosition(), Units.Radians);
    }
}
