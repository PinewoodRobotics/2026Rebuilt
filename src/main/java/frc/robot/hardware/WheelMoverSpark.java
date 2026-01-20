package frc.robot.hardware;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constant.swerve.SwerveConstants;

public class WheelMoverSpark extends WheelMoverBase {

  private final SparkMax m_driveMotor;
  private final SparkMax m_turnMotor;
  private final int driveMotorPort;
  private final SparkClosedLoopController m_drivePIDController, m_turnPIDController;
  private final CANcoder turnCANcoder;
  private final RelativeEncoder m_driveRelativeEncoder, m_rotationRelativeEncoder;

  public WheelMoverSpark(
      int driveMotorChannel,
      InvertedValue driveMotorInverted,
      int turnMotorChannel,
      InvertedValue turnMotorInverted,
      int CANCoderEncoderChannel,
      SensorDirectionValue CANCoderDirection,
      double CANCoderMagnetOffset) {
    this(
        driveMotorChannel,
        invertedValueToSparkInverted(driveMotorInverted),
        turnMotorChannel,
        invertedValueToSparkInverted(turnMotorInverted),
        CANCoderEncoderChannel,
        CANCoderDirection,
        CANCoderMagnetOffset);
  }

  public WheelMoverSpark(
      int driveMotorChannel,
      boolean driveMotorReversed,
      int turnMotorChannel,
      boolean turnMotorReversed,
      int CANCoderEncoderChannel,
      SensorDirectionValue CANCoderDirection,
      double CANCoderMagnetOffset) {
    final var c = SwerveConstants.INSTANCE;

    driveMotorPort = driveMotorChannel;
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turnMotor = new SparkMax(turnMotorChannel, MotorType.kBrushless);

    m_drivePIDController = m_driveMotor.getClosedLoopController();
    m_turnPIDController = m_turnMotor.getClosedLoopController();

    turnCANcoder = new CANcoder(CANCoderEncoderChannel);
    configureCANCoder(CANCoderDirection, CANCoderMagnetOffset);

    configureDriveMotor(driveMotorReversed, c);
    m_driveRelativeEncoder = m_driveMotor.getEncoder();

    configureTurnMotor(turnMotorReversed, c);
    m_rotationRelativeEncoder = m_turnMotor.getEncoder();
    double absRotations = turnCANcoder.getAbsolutePosition().getValueAsDouble();
    m_rotationRelativeEncoder.setPosition(wrapRotations0To1(absRotations));
  }

  private void configureCANCoder(SensorDirectionValue direction, double magnetOffset) {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = magnetOffset;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    config.MagnetSensor.SensorDirection = direction;
    turnCANcoder.getConfigurator().apply(config);
  }

  /** Configures the drive motor with PID, current limit, and encoder settings. */
  private void configureDriveMotor(boolean reversed, SwerveConstants c) {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(reversed)
        .smartCurrentLimit(c.kDriveCurrentLimit);

    final double factor = (Math.PI * c.kWheelDiameterMeters) /
        c.kDriveGearRatio;

    config.encoder
        .positionConversionFactor(factor)
        .velocityConversionFactor(factor / 60);

    config.absoluteEncoder
        .positionConversionFactor(factor)
        .velocityConversionFactor(factor / 60);

    // Ensure the closed-loop is actually using the integrated encoder and has
    // gains configured; otherwise velocity commands will be extremely weak.
    config.closedLoop
        .pid(c.kDriveP, c.kDriveI, c.kDriveD)
        .iZone(c.kDriveIZ)
        .outputRange(c.kDriveMinOutput, c.kDriveMaxOutput);

    m_driveMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /** Configures the turn motor with PID, current limit, and position wrapping. */
  private void configureTurnMotor(boolean reversed, SwerveConstants c) {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(reversed)
        .smartCurrentLimit(c.kTurnCurrentLimit);

    config.closedLoop
        .pid(c.kTurnP, c.kTurnI, c.kTurnD)
        .iZone(c.kTurnIZ)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(0.0)
        .positionWrappingMaxInput(1.0)
        .outputRange(c.kTurnMinOutput, c.kTurnMaxOutput);

    // Turning encoder is configured to report module rotations (not radians):
    // Spark native unit is motor rotations; this factor converts to module
    // rotations.
    config.encoder.positionConversionFactor(c.kTurnConversionFactor);
    config.encoder.velocityConversionFactor(c.kTurnConversionFactor / 60.0);

    m_turnMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /***************************************************************************************************/

  @Override
  public Angle getAngle() {
    // Turn encoder reports module rotations; negate to match the project
    // convention.
    return Angle.ofRelativeUnits(m_rotationRelativeEncoder.getPosition(), Units.Rotations);
  }

  @Override
  public LinearVelocity getSpeed() {
    return LinearVelocity.ofRelativeUnits(-m_driveRelativeEncoder.getVelocity(), Units.MetersPerSecond);
  }

  @Override
  public Distance getDistance() {
    return Distance.ofRelativeUnits(-m_driveRelativeEncoder.getPosition(), Units.Meters);
  }

  /***************************************************************************************************/

  @Override
  protected void setSpeed(LinearVelocity mpsSpeed) {
    final var c = SwerveConstants.INSTANCE;
    final double requestedMps = mpsSpeed.in(Units.MetersPerSecond);

    final double wheelCircumference = Math.PI * c.kWheelDiameterMeters;
    final double wheelRps = wheelCircumference == 0.0 ? 0.0 : requestedMps / wheelCircumference;
    double ffVolts = c.kDriveV * wheelRps;
    // Clamp to reasonable motor voltage.
    ffVolts = Math.max(-12.0, Math.min(12.0, ffVolts));

    m_drivePIDController.setReference(
        requestedMps,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts);
  }

  @Override
  protected void turnWheel(Angle newRotationRad) {
    double rotations = newRotationRad.in(Units.Radians) / (2.0 * Math.PI);
    rotations = wrapRotations0To1(rotations);
    m_turnPIDController.setReference(
        rotations,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        0.0);
  }

  @Override
  public void drive(Angle angle, LinearVelocity speed) {
    logEverything(speed, angle);
    setSpeed(speed);
    turnWheel(angle);
  }

  @Override
  public double getCurrentAngle() {
    return getAngle().in(Units.Radians);
  }

  @Override
  public Rotation2d getRotation2d() {
    // Rotation2d will convert the Angle to radians internally.
    return new Rotation2d(-getAngle().in(Units.Radians));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDistance().in(Units.Meters),
        getRotation2d());
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(getSpeed().in(Units.MetersPerSecond), getRotation2d());
  }

  @Override
  public void reset() {
    m_rotationRelativeEncoder.setPosition(0);
    m_driveRelativeEncoder.setPosition(0);
  }

  private static boolean invertedValueToSparkInverted(InvertedValue v) {
    // Treat "counterclockwise positive" as "invert output" for SparkMax.
    return v == InvertedValue.CounterClockwise_Positive;
  }

  /** Wrap an angle in rotations into [0, 1). */
  private static double wrapRotations0To1(double rotations) {
    // Java % keeps the sign; normalize explicitly.
    rotations %= 1.0;
    if (rotations < 0.0) {
      rotations += 1.0;
    }
    return rotations;
  }

  private void logEverything(LinearVelocity requestedMps, Angle requestedAngle) {
    String base = "wheels/" + driveMotorPort + "/";
    LinearVelocity actualMps = getSpeed();
    Angle actualAngle = getAngle();
    Distance actualDistance = getDistance();

    // Requested setpoints
    Logger.recordOutput(base + "requested/speedMps", requestedMps);
    Logger.recordOutput(base + "requested/angleDeg", requestedAngle);
    Logger.recordOutput(base + "requested/angleRad", requestedAngle);
    // Also log numeric forms so we can debug optimizer/inversion issues quickly.
    Logger.recordOutput(base + "requested/speedMpsValue", requestedMps.in(Units.MetersPerSecond));
    Logger.recordOutput(base + "requested/angleDegValue", requestedAngle.in(Units.Degrees));
    Logger.recordOutput(base + "requested/angleRadValue", requestedAngle.in(Units.Radians));

    // Actual (encoder-derived) module values
    Logger.recordOutput(base + "actual/speedMps", actualMps.in(Units.MetersPerSecond));
    Logger.recordOutput(base + "actual/distanceM", actualDistance.in(Units.Meters));
    Logger.recordOutput(base + "actual/angleDeg", actualAngle.in(Units.Degrees));
    Logger.recordOutput(base + "actual/angleRad", actualAngle.in(Units.Radians));

    // Drive encoder (configured to meters and m/s)
    Logger.recordOutput(base + "driveEncoder/positionM", m_driveRelativeEncoder.getPosition());
    Logger.recordOutput(base + "driveEncoder/velocityMps", m_driveRelativeEncoder.getVelocity());

    // Turn encoder (configured to module rotations and rotations/sec)
    Logger.recordOutput(base + "turnEncoder/positionRot", m_rotationRelativeEncoder.getPosition());
    Logger.recordOutput(base + "turnEncoder/velocityRotPerSec", m_rotationRelativeEncoder.getVelocity());

    // Absolute CANCoder (CTRE reports rotations [0,1) unless configured otherwise)
    double cancoderRot = turnCANcoder.getAbsolutePosition().getValueAsDouble();
    Logger.recordOutput(base + "cancoder/absRotations", cancoderRot);
    Logger.recordOutput(base + "cancoder/absDegrees", cancoderRot * 360.0);
    Logger.recordOutput(base + "cancoder/absRadians", cancoderRot * 2.0 * Math.PI);
  }
}
