package frc.robot.subsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constant.ShooterConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem {
    private final SparkMax m_shooterMotor;
    private final SparkClosedLoopController closedLoopController;
    private final AbsoluteEncoder absoluteEncoder;

    public ShooterSubsystem(int canId, MotorType motorType) {
        this.m_shooterMotor = new SparkMax(canId, motorType);
        this.closedLoopController = m_shooterMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(ShooterConstants.kShooterReversed)
                .smartCurrentLimit(ShooterConstants.kShooterCurrentLimit);

        double factor = (2 * ShooterConstants.kWheelRadius * Math.PI)
                / ShooterConstants.kShooterMotorRotationsPerRotation;

        config.encoder.velocityConversionFactor(factor / 60);
        config.absoluteEncoder.positionConversionFactor(factor / 60);

        // Ensure the closed-loop is actually using the integrated encoder and has
        // gains configured; otherwise velocity commands will be extremely weak.
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD)
                .iZone(ShooterConstants.kShooterIZ);

        m_shooterMotor.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        this.absoluteEncoder = m_shooterMotor.getAbsoluteEncoder();
    }

    /**
     * Set the shooter velocity in meters per second.
     * 
     * @param velocity The velocity to set the shooter to.
     * @return the time in ms it will take to reach the velocity
     **/
    public int setShooterVelocity(LinearVelocity velocity) {
        closedLoopController.setReference(velocity.in(Units.MetersPerSecond), ControlType.kVelocity);
        return 0;
    }

    /**
     * Get the current shooter velocity in meters per second.
     * 
     * @return the current shooter velocity
     **/
    public LinearVelocity getCurrentShooterVelocity() {
        return LinearVelocity.ofRelativeUnits(absoluteEncoder.getVelocity(), Units.MetersPerSecond);
    }
}
