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
import frc.robot.constant.BotConstants;
import frc.robot.constant.BotConstants.RobotVariant;

/**
 * Swerve constants façade.
 * <p>
 * We keep <b>two robot-specific constant sets</b> (ABot and BBot), but all code
 * should consume the <b>active</b> set via {@link #INSTANCE}.
 */
public final class SwerveConstants {
  public Translation2d rearLeftTranslation;
  public Translation2d rearRightTranslation;
  public Translation2d frontRightTranslation;
  public Translation2d frontLeftTranslation;

  public LinearVelocity kMaxSpeed;
  public LinearAcceleration kMaxLinearAcceleration;
  public double kMaxLinearJerk;

  public AngularVelocity kMaxTurnSpeed;
  public AngularAcceleration kMaxTurnAcceleration;
  public double kMaxTurnJerk;

  public Current kTurnCurrentLimit;
  public Current kDriveCurrentLimit;

  // ------------------------------------------------------- driving motor ports
  public int kFrontLeftDriveMotorPort;
  public int kFrontRightDriveMotorPort;
  public int kRearLeftDriveMotorPort;
  public int kRearRightDriveMotorPort;
  public InvertedValue kFrontLeftDriveMotorReversed;
  public InvertedValue kRearLeftDriveMotorReversed;
  public InvertedValue kFrontRightDriveMotorReversed;
  public InvertedValue kRearRightDriveMotorReversed;
  // ------------------------------------------------------- driving motor ports

  // ------------------------------------------------------- turning motor ports
  public int kFrontLeftTurningMotorPort;
  public int kFrontRightTurningMotorPort;
  public int kRearLeftTurningMotorPort;
  public int kRearRightTurningMotorPort;
  public InvertedValue kFrontLeftTurningMotorReversed;
  public InvertedValue kFrontRightTurningMotorReversed;
  public InvertedValue kRearLeftTurningMotorReversed;
  public InvertedValue kRearRightTurningMotorReversed;
  // ------------------------------------------------------- turning motor ports

  // ------------------------------------------------------- CANCoder ports
  public int kFrontLeftCANcoderPort;
  public int kFrontRightCANcoderPort;
  public int kRearLeftCANcoderPort;
  public int kRearRightCANcoderPort;
  public SensorDirectionValue kFrontLeftCANcoderDirection;
  public SensorDirectionValue kFrontRightCANcoderDirection;
  public SensorDirectionValue kRearLeftCANcoderDirection;
  public SensorDirectionValue kRearRightCANcoderDirection;
  public double kFrontLeftCANcoderMagnetOffset;
  public double kFrontRightCANcoderMagnetOffset;
  public double kRearLeftCANcoderMagnetOffset;
  public double kRearRightCANcoderMagnetOffset;
  // ------------------------------------------------------- CANCoder ports

  public Distance kWheelDiameter;

  // PID values for the driving
  public double kDriveP;
  public double kDriveI;
  public double kDriveD;
  public double kDriveIZ;
  public Voltage kDriveV;
  public double kDriveStatorLimit;
  public Current kDriveSupplyLimit;
  public double kDriveMinOutput;
  public double kDriveMaxOutput;

  // PID values for the turning
  public double kTurnP;
  public double kTurnI;
  public double kTurnD;
  public double kTurnIZ;
  public double kTurnMinOutput;
  public double kTurnMaxOutput;
  public int kTurnStatorLimit;
  public double kTurnSupplyLimit;

  public double kTurnConversionFactor;
  public double kDriveConversionFactor;

  /** ABot (TalonFX) constants. */
  public static final SwerveConstants ABOT = fromTalonFX();

  /** BBot (Spark) constants. */
  public static final SwerveConstants BBOT = fromSpark();

  /** Active constants for the currently-selected robot variant. */
  public static final SwerveConstants INSTANCE = BotConstants.robotType == RobotVariant.BBOT ? BBOT : ABOT;

  private static SwerveConstants fromSpark() {
    final SwerveConstants c = new SwerveConstants();

    c.rearLeftTranslation = SwerveConstantsSpark.rearLeftTranslation;
    c.rearRightTranslation = SwerveConstantsSpark.rearRightTranslation;
    c.frontRightTranslation = SwerveConstantsSpark.frontRightTranslation;
    c.frontLeftTranslation = SwerveConstantsSpark.frontLeftTranslation;

    c.kMaxSpeed = SwerveConstantsSpark.kMaxSpeed;
    c.kMaxLinearAcceleration = SwerveConstantsSpark.kMaxLinearAcceleration;
    c.kMaxLinearJerk = SwerveConstantsSpark.kMaxLinearJerk;

    c.kMaxTurnSpeed = SwerveConstantsSpark.kMaxTurnSpeed;
    c.kMaxTurnAcceleration = SwerveConstantsSpark.kMaxTurnAcceleration;
    c.kMaxTurnJerk = SwerveConstantsSpark.kMaxTurnJerk;

    c.kTurnCurrentLimit = SwerveConstantsSpark.kTurnCurrentLimit;
    c.kDriveCurrentLimit = SwerveConstantsSpark.kDriveCurrentLimit;

    c.kFrontLeftDriveMotorPort = SwerveConstantsSpark.kFrontLeftDriveMotorPort;
    c.kFrontRightDriveMotorPort = SwerveConstantsSpark.kFrontRightDriveMotorPort;
    c.kRearLeftDriveMotorPort = SwerveConstantsSpark.kRearLeftDriveMotorPort;
    c.kRearRightDriveMotorPort = SwerveConstantsSpark.kRearRightDriveMotorPort;

    c.kFrontLeftDriveMotorReversed = SwerveConstantsSpark.kFrontLeftDriveMotorReversed;
    c.kRearLeftDriveMotorReversed = SwerveConstantsSpark.kRearLeftDriveMotorReversed;
    c.kFrontRightDriveMotorReversed = SwerveConstantsSpark.kFrontRightDriveMotorReversed;
    c.kRearRightDriveMotorReversed = SwerveConstantsSpark.kRearRightDriveMotorReversed;

    c.kFrontLeftTurningMotorPort = SwerveConstantsSpark.kFrontLeftTurningMotorPort;
    c.kFrontRightTurningMotorPort = SwerveConstantsSpark.kFrontRightTurningMotorPort;
    c.kRearLeftTurningMotorPort = SwerveConstantsSpark.kRearLeftTurningMotorPort;
    c.kRearRightTurningMotorPort = SwerveConstantsSpark.kRearRightTurningMotorPort;

    c.kFrontLeftTurningMotorReversed = SwerveConstantsSpark.kFrontLeftTurningMotorReversed;
    c.kFrontRightTurningMotorReversed = SwerveConstantsSpark.kFrontRightTurningMotorReversed;
    c.kRearLeftTurningMotorReversed = SwerveConstantsSpark.kRearLeftTurningMotorReversed;
    c.kRearRightTurningMotorReversed = SwerveConstantsSpark.kRearRightTurningMotorReversed;

    c.kFrontLeftCANcoderPort = SwerveConstantsSpark.kFrontLeftCANcoderPort;
    c.kFrontRightCANcoderPort = SwerveConstantsSpark.kFrontRightCANcoderPort;
    c.kRearLeftCANcoderPort = SwerveConstantsSpark.kRearLeftCANcoderPort;
    c.kRearRightCANcoderPort = SwerveConstantsSpark.kRearRightCANcoderPort;

    c.kFrontLeftCANcoderDirection = SwerveConstantsSpark.kFrontLeftCANcoderDirection;
    c.kFrontRightCANcoderDirection = SwerveConstantsSpark.kFrontRightCANcoderDirection;
    c.kRearLeftCANcoderDirection = SwerveConstantsSpark.kRearLeftCANcoderDirection;
    c.kRearRightCANcoderDirection = SwerveConstantsSpark.kRearRightCANcoderDirection;

    c.kFrontLeftCANcoderMagnetOffset = SwerveConstantsSpark.kFrontLeftCANcoderMagnetOffset;
    c.kFrontRightCANcoderMagnetOffset = SwerveConstantsSpark.kFrontRightCANcoderMagnetOffset;
    c.kRearLeftCANcoderMagnetOffset = SwerveConstantsSpark.kRearLeftCANcoderMagnetOffset;
    c.kRearRightCANcoderMagnetOffset = SwerveConstantsSpark.kRearRightCANcoderMagnetOffset;

    c.kWheelDiameter = SwerveConstantsSpark.kWheelDiameter;

    c.kDriveP = SwerveConstantsSpark.kDriveP;
    c.kDriveI = SwerveConstantsSpark.kDriveI;
    c.kDriveD = SwerveConstantsSpark.kDriveD;
    c.kDriveIZ = SwerveConstantsSpark.kDriveIZ;
    c.kDriveV = SwerveConstantsSpark.kDriveV;
    c.kDriveMinOutput = SwerveConstantsSpark.kDriveMinOutput;
    c.kDriveMaxOutput = SwerveConstantsSpark.kDriveMaxOutput;
    c.kDriveStatorLimit = SwerveConstantsSpark.kDriveStatorLimit;
    c.kDriveSupplyLimit = SwerveConstantsSpark.kDriveSupplyLimit;

    c.kTurnP = SwerveConstantsSpark.kTurnP;
    c.kTurnI = SwerveConstantsSpark.kTurnI;
    c.kTurnD = SwerveConstantsSpark.kTurnD;
    c.kTurnIZ = SwerveConstantsSpark.kTurnIZ;
    c.kTurnMinOutput = SwerveConstantsSpark.kTurnMinOutput;
    c.kTurnMaxOutput = SwerveConstantsSpark.kTurnMaxOutput;
    c.kTurnStatorLimit = SwerveConstantsSpark.kTurnStatorLimit;
    c.kTurnSupplyLimit = SwerveConstantsSpark.kTurnSupplyLimit;

    c.kTurnConversionFactor = SwerveConstantsSpark.kTurnConversionFactor;
    c.kDriveConversionFactor = SwerveConstantsSpark.kDriveConversionFactor;

    return c;
  }

  private static SwerveConstants fromTalonFX() {
    final SwerveConstants c = new SwerveConstants();

    c.rearLeftTranslation = SwerveConstantsTalonFX.rearLeftTranslation;
    c.rearRightTranslation = SwerveConstantsTalonFX.rearRightTranslation;
    c.frontRightTranslation = SwerveConstantsTalonFX.frontRightTranslation;
    c.frontLeftTranslation = SwerveConstantsTalonFX.frontLeftTranslation;

    c.kMaxSpeed = Units.MetersPerSecond.of(SwerveConstantsTalonFX.tempMaxSpeed);
    c.kMaxLinearAcceleration = SwerveConstantsTalonFX.kMaxLinearAcceleration;
    c.kMaxLinearJerk = SwerveConstantsTalonFX.kMaxLinearJerk;

    c.kMaxTurnSpeed = SwerveConstantsTalonFX.kMaxTurnSpeed;
    c.kMaxTurnAcceleration = SwerveConstantsTalonFX.kMaxTurnAcceleration;
    c.kMaxTurnJerk = SwerveConstantsTalonFX.kMaxTurnJerk;

    c.kTurnCurrentLimit = SwerveConstantsTalonFX.kTurnCurrentLimit;
    c.kDriveCurrentLimit = SwerveConstantsTalonFX.kDriveCurrentLimit;

    c.kFrontLeftDriveMotorPort = SwerveConstantsTalonFX.kFrontLeftDriveMotorPort;
    c.kFrontRightDriveMotorPort = SwerveConstantsTalonFX.kFrontRightDriveMotorPort;
    c.kRearLeftDriveMotorPort = SwerveConstantsTalonFX.kRearLeftDriveMotorPort;
    c.kRearRightDriveMotorPort = SwerveConstantsTalonFX.kRearRightDriveMotorPort;

    c.kFrontLeftDriveMotorReversed = SwerveConstantsTalonFX.kFrontLeftDriveMotorReversed;
    c.kRearLeftDriveMotorReversed = SwerveConstantsTalonFX.kRearLeftDriveMotorReversed;
    c.kFrontRightDriveMotorReversed = SwerveConstantsTalonFX.kFrontRightDriveMotorReversed;
    c.kRearRightDriveMotorReversed = SwerveConstantsTalonFX.kRearRightDriveMotorReversed;

    c.kFrontLeftTurningMotorPort = SwerveConstantsTalonFX.kFrontLeftTurningMotorPort;
    c.kFrontRightTurningMotorPort = SwerveConstantsTalonFX.kFrontRightTurningMotorPort;
    c.kRearLeftTurningMotorPort = SwerveConstantsTalonFX.kRearLeftTurningMotorPort;
    c.kRearRightTurningMotorPort = SwerveConstantsTalonFX.kRearRightTurningMotorPort;

    c.kFrontLeftTurningMotorReversed = SwerveConstantsTalonFX.kFrontLeftTurningMotorReversed;
    c.kFrontRightTurningMotorReversed = SwerveConstantsTalonFX.kFrontRightTurningMotorReversed;
    c.kRearLeftTurningMotorReversed = SwerveConstantsTalonFX.kRearLeftTurningMotorReversed;
    c.kRearRightTurningMotorReversed = SwerveConstantsTalonFX.kRearRightTurningMotorReversed;

    c.kFrontLeftCANcoderPort = SwerveConstantsTalonFX.kFrontLeftCANcoderPort;
    c.kFrontRightCANcoderPort = SwerveConstantsTalonFX.kFrontRightCANcoderPort;
    c.kRearLeftCANcoderPort = SwerveConstantsTalonFX.kRearLeftCANcoderPort;
    c.kRearRightCANcoderPort = SwerveConstantsTalonFX.kRearRightCANcoderPort;

    c.kFrontLeftCANcoderDirection = SwerveConstantsTalonFX.kFrontLeftCANcoderDirection;
    c.kFrontRightCANcoderDirection = SwerveConstantsTalonFX.kFrontRightCANcoderDirection;
    c.kRearLeftCANcoderDirection = SwerveConstantsTalonFX.kRearLeftCANcoderDirection;
    c.kRearRightCANcoderDirection = SwerveConstantsTalonFX.kRearRightCANcoderDirection;

    c.kFrontLeftCANcoderMagnetOffset = SwerveConstantsTalonFX.kFrontLeftCANcoderMagnetOffset;
    c.kFrontRightCANcoderMagnetOffset = SwerveConstantsTalonFX.kFrontRightCANcoderMagnetOffset;
    c.kRearLeftCANcoderMagnetOffset = SwerveConstantsTalonFX.kRearLeftCANcoderMagnetOffset;
    c.kRearRightCANcoderMagnetOffset = SwerveConstantsTalonFX.kRearRightCANcoderMagnetOffset;

    c.kWheelDiameter = SwerveConstantsTalonFX.kWheelDiameter;

    c.kDriveP = SwerveConstantsTalonFX.kDriveP;
    c.kDriveI = SwerveConstantsTalonFX.kDriveI;
    c.kDriveD = SwerveConstantsTalonFX.kDriveD;
    c.kDriveIZ = SwerveConstantsTalonFX.kDriveIZ;
    c.kDriveV = SwerveConstantsTalonFX.kDriveV;
    c.kDriveMinOutput = SwerveConstantsTalonFX.kDriveMinOutput;
    c.kDriveMaxOutput = SwerveConstantsTalonFX.kDriveMaxOutput;
    c.kDriveStatorLimit = SwerveConstantsTalonFX.kDriveStatorLimit;
    c.kDriveSupplyLimit = SwerveConstantsTalonFX.kDriveSupplyLimit;

    c.kTurnP = SwerveConstantsTalonFX.kTurnP;
    c.kTurnI = SwerveConstantsTalonFX.kTurnI;
    c.kTurnD = SwerveConstantsTalonFX.kTurnD;
    c.kTurnIZ = SwerveConstantsTalonFX.kTurnIZ;
    c.kTurnMinOutput = SwerveConstantsTalonFX.kTurnMinOutput;
    c.kTurnMaxOutput = SwerveConstantsTalonFX.kTurnMaxOutput;
    c.kTurnStatorLimit = SwerveConstantsTalonFX.kTurnStatorLimit;
    c.kTurnSupplyLimit = SwerveConstantsTalonFX.kTurnSupplyLimit;

    c.kTurnConversionFactor = SwerveConstantsTalonFX.kTurnConversionFactor;

    // For TalonFX, treat this as motor rotations per wheel rotation (gear ratio).
    c.kDriveConversionFactor = SwerveConstantsTalonFX.kDriveGearRatio;

    return c;
  }
}
