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

public class SwerveConstantsTalonFX {
  public static final Translation2d rearLeftTranslation = new Translation2d(
      -0.3429,
      0.3429);

  public static final Translation2d rearRightTranslation = new Translation2d(
      -0.3429,
      -0.3429);

  public static final Translation2d frontRightTranslation = new Translation2d(
      0.3429,
      -0.3429);

  public static final Translation2d frontLeftTranslation = new Translation2d(
      0.3429,
      0.3429);

  public static final LinearVelocity kMaxSpeed = Units.MetersPerSecond.of(0);
  public static final LinearAcceleration kMaxLinearAcceleration = Units.MetersPerSecondPerSecond.of(0);
  /** Units: meters/sec^3 */
  public static final double kMaxLinearJerk = 20.0;

  public static final Current kTurnCurrentLimit = Units.Amps.of(30);
  public static final Current kDriveCurrentLimit = Units.Amps.of(30);

  // the driving motor ports
  public static final int kFrontLeftDriveMotorPort = 7;
  public static final int kFrontRightDriveMotorPort = 13;
  public static final int kRearLeftDriveMotorPort = 9;
  public static final int kRearRightDriveMotorPort = 11;

  // whether the driving encoders are flipped
  public static final InvertedValue kFrontLeftDriveMotorReversed = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue kRearLeftDriveMotorReversed = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue kFrontRightDriveMotorReversed = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue kRearRightDriveMotorReversed = InvertedValue.Clockwise_Positive;

  // the turning motor ports
  public static final int kFrontLeftTurningMotorPort = 6;
  public static final int kFrontRightTurningMotorPort = 12;
  public static final int kRearLeftTurningMotorPort = 8;
  public static final int kRearRightTurningMotorPort = 10;

  // Whether the turning motors are flipped. These are module-local hardware
  // settings and should not change when the robot/world coordinate frame changes.
  public static final InvertedValue kFrontLeftTurningMotorReversed = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue kFrontRightTurningMotorReversed = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue kRearLeftTurningMotorReversed = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue kRearRightTurningMotorReversed = InvertedValue.CounterClockwise_Positive;

  // the CANCoder turning encoder ports - updated 2/12/24
  public static final int kFrontLeftCANcoderPort = 2;
  public static final int kFrontRightCANcoderPort = 5;
  public static final int kRearLeftCANcoderPort = 3;
  public static final int kRearRightCANcoderPort = 4;

  // whether the turning CANCoders are flipped

  public static final SensorDirectionValue kFrontLeftCANcoderDirection = SensorDirectionValue.CounterClockwise_Positive;
  public static final SensorDirectionValue kFrontRightCANcoderDirection = SensorDirectionValue.CounterClockwise_Positive;
  public static final SensorDirectionValue kRearLeftCANcoderDirection = SensorDirectionValue.CounterClockwise_Positive;
  public static final SensorDirectionValue kRearRightCANcoderDirection = SensorDirectionValue.CounterClockwise_Positive;

  // Magnetic offsets for the CANCoders.
  // These are calibrated physical zeros for the modules, not field-frame values.
  // you can find these by connecting to the RoboRIO by USB on the drive station,
  // opening the Phoenix Tuner app, and taking snapshots of
  // the rotational values of the CANCoders while in they are in the forward state
  // units: rotations
  public static final double kFrontLeftCANcoderMagnetOffset = -0.184;
  public static final double kFrontRightCANcoderMagnetOffset = -0.285;
  public static final double kRearLeftCANcoderMagnetOffset = 0.317;
  public static final double kRearRightCANcoderMagnetOffset = 0.062;

  // stats used by SwerveSubsystem for math
  public static final Distance kWheelDiameter = Units.Meters.of(0.089);
  public static final double kDriveBaseWidth = 0.66;
  public static final double kDriveBaseLength = 0.66;

  // stats used by SwerveSubsystem for deadbanding
  public static final double kXSpeedDeadband = 0.05;
  public static final double kXSpeedMinValue = 0;
  public static final double kYSpeedDeadband = 0.05;
  public static final double kYSpeedMinValue = 0;
  public static final double kRotDeadband = 0.05;
  public static final double kRotMinValue = 0;

  public static final boolean kFieldRelative = true;
  public static final boolean kOptimizeAngles = true;
  public static final boolean kPIDDirection = true;
  public static final double kDirectionP = 2;
  public static final double kDirectionI = 0.004;
  public static final double kDirectionD = 0.02;
  public static final double kDirectionMultiplier = 0.01;

  // PID values for the driving
  public static final double kDriveP = 0.5;
  public static final double kDriveI = 1;
  public static final double kDriveD = 0;
  public static final double kDriveIZ = 0;
  public static final double kDriveFF = 0;
  public static final double kDriveV = 0.6; // Velocity feedforward - critical for velocity control
  public static final double kDriveMinOutput = -1;
  public static final double kDriveMaxOutput = 1;

  // multiplies the output speed of all of the drive motors, ALWAYS (0, 1).
  public static final double kDefaultSpeedMultiplier = 0.75;
  public static final double kRotationSpeedMultiplier = 0.5;
  public static final double kIntakeSpeedMultiplier = kDefaultSpeedMultiplier;
  public static final double kAutonSpeedMultiplier = 0.5;

  public static final double kDriveMaxRPM = 5700;
  public static final Current kDriveStatorLimit = Units.Amps.of(70); // TEMP
  public static final Current kDriveSupplyLimit = Units.Amps.of(30); // TEMP

  // PID values for the turning
  public static final double kTurnP = 1.5 * 12;
  public static final double kTurnI = 0.0015 * 12;
  public static final double kTurnD = 0.12 * 12;
  public static final double kTurnIZ = 0;
  public static final double kTurnFF = 0;
  public static final double kTurnMinOutput = -1;
  public static final double kTurnMaxOutput = 1;
  public static final int kTurnStatorLimit = 30; // TEMP
  public static final double kTurnSupplyLimit = 30; // TEMP

  public static final double kHeadingP = 2;
  public static final double kHeadingI = 0.004;
  public static final double kHeadingD = 0.01;

  // because the turn gearing ratio is not 1:1, we need to spin the motor many
  // times to equal one spin of the module
  // this constant is used for the position conversion factor. (every 150 turns of
  // motors is 7 rotations of the module)
  public static final double kTurnConversionFactor = 25.9;

  // because the drive gearing ratio is not 1:1, we need to spin the motor many
  // times to equal one spin of the module
  public static final double kDriveGearRatio = 4.94;
  public static final double kThursdayHackGearRatio = 0.25;
  public static final double kThursdayHackDirection = -1;

  // Motion Magic configuration for drive motors (velocity control with trapezoid
  // profiling)
  public static final double kDriveMotionMagicAcceleration = 50; // rotations/sec² - tune for smoother/faster
                                                                 // acceleration
  public static final double kDriveMotionMagicJerk = 1000; // rotations/sec³ - tune for smoother starts/stops

  // Motion Magic configuration for turn motors (position control with trapezoid
  // profiling)
  public static final AngularVelocity kTurnMotionMagicCruiseVelocity = Units.RotationsPerSecond.of(0);
  public static final AngularAcceleration kTurnMotionMagicAcceleration = Units.RotationsPerSecondPerSecond.of(0);

  public static final int kPigeonCANId = 40;
}
