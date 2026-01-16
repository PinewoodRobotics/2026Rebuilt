package frc.robot.hardware;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constant.swerve.SwerveConstants;

public class WheelMoverTalonFX extends WheelMoverBase {

  private TalonFX m_driveMotor;
  private TalonFX m_turnMotor;
  private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);
  private final int port;

  private CANcoder turnCANcoder;

  public WheelMoverTalonFX(
      int driveMotorChannel,
      InvertedValue driveMotorReversed,
      int turnMotorChannel,
      InvertedValue turnMotorReversed,
      int CANCoderEncoderChannel,
      SensorDirectionValue CANCoderDirection,
      double CANCoderMagnetOffset) {
    final var c = SwerveConstants.INSTANCE;
    this.port = driveMotorChannel;
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turnMotor = new TalonFX(turnMotorChannel);

    turnCANcoder = new CANcoder(CANCoderEncoderChannel);
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = -CANCoderMagnetOffset;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    config.MagnetSensor.SensorDirection = CANCoderDirection;
    turnCANcoder.getConfigurator().apply(config);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(driveMotorReversed)
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(
                    c.kDriveStatorLimit)
                .withSupplyCurrentLimit(
                    c.kDriveSupplyLimit))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(
                    c.kDriveGearRatio))
        .withSlot0(
            new Slot0Configs()
                .withKP(c.kDriveP)
                .withKI(c.kDriveI)
                .withKD(c.kDriveD)
                .withKV(c.kDriveV))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(c.kDriveMotionMagicAcceleration)
                .withMotionMagicJerk(c.kDriveMotionMagicJerk));

    m_driveMotor.getConfigurator().apply(driveConfig);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(turnMotorReversed)
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(
                    c.kTurnStatorLimit)
                .withSupplyCurrentLimit(
                    c.kTurnSupplyLimit))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(
                    c.kTurnConversionFactor))
        .withSlot0(
            new Slot0Configs()
                .withKP(c.kTurnP)
                .withKI(c.kTurnI)
                .withKD(c.kTurnD))
        .withClosedLoopGeneral(
            new ClosedLoopGeneralConfigs().withContinuousWrap(true))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(c.kTurnMotionMagicCruiseVelocity)
                .withMotionMagicAcceleration(c.kTurnMotionMagicAcceleration)
                .withMotionMagicJerk(c.kTurnMotionMagicJerk));

    m_turnMotor.getConfigurator().apply(turnConfig);
    m_turnMotor.setPosition(
        turnCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  protected void setSpeed(LinearVelocity mpsSpeed) {
    final var c = SwerveConstants.INSTANCE;
    double wheelCircumference = Math.PI * c.kWheelDiameterMeters;
    double speedMps = mpsSpeed.in(Units.MetersPerSecond);
    double wheelRps = speedMps / wheelCircumference;

    m_driveMotor.setControl(velocityRequest.withVelocity(wheelRps));
  }

  @Override
  protected void turnWheel(Angle newRotation) {
    // CTRE uses rotations for position; convert from radians for a consistent API.
    double rotations = newRotation.in(Units.Radians) / (2.0 * Math.PI);
    m_turnMotor.setControl(positionRequest.withPosition(rotations));
  }

  @Override
  public double getCurrentAngle() {
    return getAngle().in(Units.Radians);
  }

  /***************************************************************************************************/

  @Override
  public Angle getAngle() {
    // Turn sensor is configured to report module rotations; negate to match project
    // convention.
    return Angle.ofRelativeUnits(-m_turnMotor.getPosition().getValueAsDouble(), Units.Rotations);
  }

  @Override
  public LinearVelocity getSpeed() {
    return LinearVelocity.ofRelativeUnits(
        convertWheelRotationsToMeters(m_driveMotor.getVelocity().getValueAsDouble()),
        Units.MetersPerSecond);
  }

  @Override
  public Distance getDistance() {
    return Distance.ofRelativeUnits(
        convertWheelRotationsToMeters(m_driveMotor.getPosition().getValueAsDouble()),
        Units.Meters);
  }

  /***************************************************************************************************/

  /**
   * Converts wheel rotations to distance/velocity in meters
   * Note: With SensorToMechanismRatio configured, motor values are already in
   * wheel rotations
   */
  private double convertWheelRotationsToMeters(double wheelRotations) {
    return -wheelRotations * (Math.PI * SwerveConstants.INSTANCE.kWheelDiameterMeters);
  }

  public double getCANCoderAngle() {
    return turnCANcoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public Rotation2d getRotation2d() {
    return new Rotation2d(getAngle().in(Units.Radians));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDistance().in(Units.Meters),
        getRotation2d());
  }

  @Override
  public SwerveModuleState getState() {
    Logger.recordOutput("Wheels/" + port + "/value", m_driveMotor.getPosition().getValueAsDouble());
    return new SwerveModuleState(
        getSpeed().in(Units.MetersPerSecond),
        getRotation2d());
  }

  @Override
  public void reset() {
    m_turnMotor.setPosition(0);
    m_driveMotor.setPosition(0);
  }

  private void logEverything(LinearVelocity requestedMps, Angle requestedAngle) {
    String base = "Wheels/" + port + "/";
    LinearVelocity mpsSpeed = getSpeed();
    Angle newRotationRad = getAngle();
    Distance distance = getDistance();

    Logger.recordOutput(base + "requestedMps", mpsSpeed.in(Units.MetersPerSecond));
    Logger.recordOutput(base + "requestedAngle", requestedAngle.in(Units.Degrees));
    Logger.recordOutput(base + "requestedDistance", distance.in(Units.Meters));
  }
}
