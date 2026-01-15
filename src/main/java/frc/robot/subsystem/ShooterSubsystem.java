package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.ShooterConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax m_shooterMotor;
    private final SparkClosedLoopController closedLoopController;
    private final AbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder relativeEncoder;

    private static ShooterSubsystem instance;

    public static ShooterSubsystem GetInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem(ShooterConstants.kShooterCanId, ShooterConstants.kShooterMotorType);
        }

        return instance;
    }

    public ShooterSubsystem(int canId, MotorType motorType) {
        this.m_shooterMotor = new SparkMax(canId, motorType);
        this.closedLoopController = m_shooterMotor.getClosedLoopController();
        this.relativeEncoder = m_shooterMotor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(ShooterConstants.kShooterReversed)
                .smartCurrentLimit(ShooterConstants.kShooterCurrentLimit);

        double factor = (2 * ShooterConstants.kWheelRadius * Math.PI)
                / ShooterConstants.kShooterMotorRotationsPerRotation;

        // Velocity is m/s (Spark reports RPM by default).
        config.encoder.velocityConversionFactor(factor / 60.0);
        config.absoluteEncoder.velocityConversionFactor(factor / 60.0);

        // Ensure the closed-loop is actually using the integrated encoder and has
        // gains configured; otherwise velocity commands will be extremely weak.
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD)
                .iZone(ShooterConstants.kShooterIZ);

        // These limits are enforced when using kMAXMotionVelocityControl.
        config.closedLoop.maxMotion
                .maxVelocity(ShooterConstants.kShooterMaxVelocity.in(Units.MetersPerSecond))
                .maxAcceleration(ShooterConstants.kShooterMaxAcceleration.in(Units.MetersPerSecondPerSecond));

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
        closedLoopController.setReference(velocity.in(Units.MetersPerSecond), ControlType.kMAXMotionVelocityControl);
        return 0;
    }

    /**
     * Get the current shooter velocity in meters per second.
     * 
     * @return the current shooter velocity
     **/
    public LinearVelocity getCurrentShooterVelocity() {
        return LinearVelocity.ofRelativeUnits(relativeEncoder.getVelocity(), Units.MetersPerSecond);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/Velocity", getCurrentShooterVelocity().in(Units.MetersPerSecond));

        Logger.recordOutput("Shooter/RawAbsoluteEncoderVelocity", absoluteEncoder.getVelocity());

        Logger.recordOutput("Shooter/AppliedOutput", m_shooterMotor.getAppliedOutput());
        Logger.recordOutput("Shooter/BusVoltage", m_shooterMotor.getBusVoltage());
        Logger.recordOutput("Shooter/OutputCurrent", m_shooterMotor.getOutputCurrent());
    }
}
