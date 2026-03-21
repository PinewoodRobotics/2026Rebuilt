package frc.robot.hardware;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
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
    config.MagnetSensor.MagnetOffset = CANCoderMagnetOffset;
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
                    c.kDriveConversionFactor))
        .withSlot0(
            new Slot0Configs()
                .withKP(c.kDriveP)
                .withKI(c.kDriveI)
                .withKD(c.kDriveD)
                .withKV(c.kDriveV))
        .withMotionMagic(
            new MotionMagicConfigs()
                // Phoenix expects mechanism rotations/sec; we treat the mechanism as the
                // wheel (SensorToMechanismRatio is configured to wheel rotations).
                .withMotionMagicCruiseVelocity(maxWheelRps(c))
                .withMotionMagicAcceleration(maxWheelRpsPerSec(c))
                .withMotionMagicJerk(maxWheelRpsPerSec2(c)));

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
                // CTRE expects motor rotations per mechanism rotation (module rotation).
                // Our project constant is module rotations per motor rotation.
                .withSensorToMechanismRatio(c.kTurnConversionFactor))
        .withSlot0(
            new Slot0Configs()
                .withKP(c.kTurnP)
                .withKI(c.kTurnI)
                .withKD(c.kTurnD))
        .withClosedLoopGeneral(
            new ClosedLoopGeneralConfigs().withContinuousWrap(true))
        .withMotionMagic(
            new MotionMagicConfigs()
                // Phoenix expects mechanism rotations/sec (module rotations/sec).
                .withMotionMagicCruiseVelocity(c.kMaxTurnSpeed)
                .withMotionMagicAcceleration(c.kMaxTurnAcceleration)
                .withMotionMagicJerk(c.kMaxTurnJerk));

    m_turnMotor.getConfigurator().apply(turnConfig);

    m_turnMotor.setPosition(
        turnCANcoder.getAbsolutePosition().getValueAsDouble());

    AudioConfigs audioConfigs = new AudioConfigs().withAllowMusicDurDisable(true);
    m_driveMotor.getConfigurator().apply(audioConfigs);
    m_turnMotor.getConfigurator().apply(audioConfigs);
  }

  @Override
  protected void setSpeed(LinearVelocity mpsSpeed) {
    final var c = SwerveConstants.INSTANCE;
    double wheelCircumference = Math.PI * c.kWheelDiameter.in(Units.Meters);
    double speedMps = mpsSpeed.in(Units.MetersPerSecond);
    double wheelRps = speedMps / wheelCircumference;

    m_driveMotor.setControl(velocityRequest.withVelocity(wheelRps));
  }

  @Override
  protected void turnWheel(Angle newRotation) {
    m_turnMotor.setControl(positionRequest.withPosition(newRotation));
  }

  @Override
  public void drive(Angle angle, LinearVelocity speed) {
    setSpeed(speed);
    turnWheel(angle);

    logEverything(speed, angle);
  }

  @Override
  public double getCurrentAngle() {
    return getAngle().in(Units.Radians);
  }

  /***************************************************************************************************/

  @Override
  public Angle getAngle() {
    return wrapAngle(m_turnMotor.getPosition().getValue());
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

  private Angle wrapAngle(Angle angle) {
    double radians = angle.in(Units.Radians);
    double wrappedRadians = Math.atan2(Math.sin(radians), Math.cos(radians));
    return Units.Radians.of(wrappedRadians);
  }

  /**
   * Converts wheel rotations to distance/velocity in meters
   * Note: With SensorToMechanismRatio configured, motor values are already in
   * wheel rotations
   */
  private double convertWheelRotationsToMeters(double wheelRotations) {
    return wheelRotations * (Math.PI * SwerveConstants.INSTANCE.kWheelDiameter.in(Units.Meters));
  }

  public double getCANCoderAngle() {
    return turnCANcoder.getAbsolutePosition().getValueAsDouble();
  }

  public TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  public TalonFX getTurnMotor() {
    return m_turnMotor;
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

    var currentAngle = getAngle();
    var currentSpeed = getSpeed();
    var currentDistance = getDistance();
    var rawAngle = getCurrentAngle();

    Logger.recordOutput(base + "requestedMps", requestedMps.in(Units.MetersPerSecond));
    Logger.recordOutput(base + "requestedAngle", requestedAngle.in(Units.Degrees));

    Logger.recordOutput(base + "currentAngle", currentAngle.in(Units.Degrees));
    Logger.recordOutput(base + "currentSpeed", currentSpeed.in(Units.MetersPerSecond));
    Logger.recordOutput(base + "currentDistance", currentDistance.in(Units.Meters));

    Logger.recordOutput(base + "rawCurrentAngle", rawAngle);

  }

  // ***********************************************************************************************
  // ***********************************************************************************************
  // ***********************************************************************************************
  // ***********************************************************************************************

  private static double maxWheelRps(SwerveConstants c) {
    final double wheelCircumference = Math.PI * c.kWheelDiameter.in(Units.Meters);
    if (wheelCircumference == 0.0) {
      return 0.0;
    }
    return c.kMaxSpeed.in(Units.MetersPerSecond) / wheelCircumference;
  }

  private static double maxWheelRpsPerSec(SwerveConstants c) {
    final double wheelCircumference = Math.PI * c.kWheelDiameter.in(Units.Meters);
    if (wheelCircumference == 0.0) {
      return 0.0;
    }
    return c.kMaxLinearAcceleration.in(Units.MetersPerSecondPerSecond) / wheelCircumference;
  }

  private static double maxWheelRpsPerSec2(SwerveConstants c) {
    final double wheelCircumference = Math.PI * c.kWheelDiameter.in(Units.Meters);
    if (wheelCircumference == 0.0) {
      return 0.0;
    }
    // c.kMaxLinearJerk is in meters/sec^3.
    return c.kMaxLinearJerk / wheelCircumference;
  }
}
