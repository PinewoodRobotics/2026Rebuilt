package frc.robot.constant.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * BBot (Spark) swerve constants.
 * <p>
 * Keep this file aligned with the public fields on {@link SwerveConstants}.
 */
public final class SwerveConstantsSpark {
  public static final Translation2d rearLeftTranslation = new Translation2d(
      0.38,
      0.38);

  public static final Translation2d rearRightTranslation = new Translation2d(
      0.38,
      -0.38);

  public static final Translation2d frontRightTranslation = new Translation2d(
      -0.38,
      -0.38);

  public static final Translation2d frontLeftTranslation = new Translation2d(
      -0.38,
      0.38);

  public static final LinearVelocity kMaxSpeed = Units.MetersPerSecond.of(2);
  public static final LinearAcceleration kMaxLinearAcceleration = Units.MetersPerSecondPerSecond.of(3.0);
  /** Units: meters/sec^3 */
  public static final double kMaxLinearJerk = 20.0;

  public static final AngularVelocity kMaxTurnSpeed = Units.RadiansPerSecond.of(Math.PI / 1.3);
  public static final AngularAcceleration kMaxTurnAcceleration = Units.RadiansPerSecondPerSecond.of(10.0);
  /** Units: radians/sec^3 */
  public static final double kMaxTurnJerk = 100.0;

  public static final Current kTurnCurrentLimit = Units.Amps.of(10);
  public static final Current kDriveCurrentLimit = Units.Amps.of(30);

  // the driving motor ports
  public static final int kFrontLeftDriveMotorPort = 12;
  public static final int kFrontRightDriveMotorPort = 25;
  public static final int kRearRightDriveMotorPort = 10;
  public static final int kRearLeftDriveMotorPort = 21;

  // whether the driving encoders are flipped
  public static final InvertedValue kFrontLeftDriveMotorReversed = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue kRearLeftDriveMotorReversed = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue kFrontRightDriveMotorReversed = InvertedValue.Clockwise_Positive;
  public static final InvertedValue kRearRightDriveMotorReversed = InvertedValue.Clockwise_Positive;

  // the turning motor ports
  public static final int kFrontLeftTurningMotorPort = 13;
  public static final int kFrontRightTurningMotorPort = 20;
  public static final int kRearRightTurningMotorPort = 23;
  public static final int kRearLeftTurningMotorPort = 22;

  // whether the turning enoders are flipped
  public static final InvertedValue kFrontLeftTurningMotorReversed = InvertedValue.Clockwise_Positive;
  public static final InvertedValue kFrontRightTurningMotorReversed = InvertedValue.Clockwise_Positive;
  public static final InvertedValue kRearLeftTurningMotorReversed = InvertedValue.Clockwise_Positive;
  public static final InvertedValue kRearRightTurningMotorReversed = InvertedValue.Clockwise_Positive;

  // the CANCoder turning encoder ports - updated 2/12/24
  public static final int kFrontLeftCANcoderPort = 1;
  public static final int kFrontRightCANcoderPort = 2;
  public static final int kRearRightCANcoderPort = 3;
  public static final int kRearLeftCANcoderPort = 4;

  // whether the turning CANCoders are flipped

  public static final SensorDirectionValue kFrontLeftCANcoderDirection = SensorDirectionValue.Clockwise_Positive;
  public static final SensorDirectionValue kFrontRightCANcoderDirection = SensorDirectionValue.Clockwise_Positive;
  public static final SensorDirectionValue kRearLeftCANcoderDirection = SensorDirectionValue.Clockwise_Positive;
  public static final SensorDirectionValue kRearRightCANcoderDirection = SensorDirectionValue.Clockwise_Positive;

  // magnetic offset for the CANCoders
  // you can find these by connecting to the RoboRIO by USB on the drive station,
  // opening the Phoenix Tuner app, and taking snapshots of
  // the rotational values of the CANCoders while in they are in the forward state
  // units: rotations
  public static final double kFrontLeftCANcoderMagnetOffset = -0.0617;
  public static final double kFrontRightCANcoderMagnetOffset = -0.0771;
  public static final double kRearRightCANcoderMagnetOffset = -0.0976;
  public static final double kRearLeftCANcoderMagnetOffset = 0.255;

  public static final Distance kWheelDiameter = Units.Meters.of(0.09);

  public static final double kDriveP = 0.01;
  public static final double kDriveI = 0;
  public static final double kDriveD = 0;
  public static final double kDriveIZ = 0;
  public static final Voltage kDriveV = Units.Volts.of(0.8); // Velocity feedforward - critical for velocity control
  public static final double kDriveMinOutput = -1;
  public static final double kDriveMaxOutput = 1;

  public static final double kDriveStatorLimit = 70; // TEMP
  public static final Current kDriveSupplyLimit = Units.Amps.of(40); // TEMP

  // PID values for the turning
  public static final double kTurnP = 1.5;
  public static final double kTurnI = 0.0015;
  public static final double kTurnD = 0.12;
  public static final double kTurnIZ = 0;
  public static final double kTurnMinOutput = -1;
  public static final double kTurnMaxOutput = 1;
  public static final int kTurnStatorLimit = 30; // TEMP
  public static final double kTurnSupplyLimit = 30; // TEMP

  /**
   * Spark encoders report motor rotations; this converts motor rotations to
   * module
   * rotations.
   */
  public static final double kTurnConversionFactor = 7.0 / 150.0;

  /** Motor rotations per wheel rotation. */
  public static final double kDriveConversionFactor = 6.75;

  private SwerveConstantsSpark() {
  }
}
