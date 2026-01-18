package frc.robot.constant.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public Pose2d headingControl;

  public double kMaxSpeedMPSNormElevator;
  public double kMaxSpeedMPSTopElevator;
  public double tempMaxSpeed;
  public int kTurnCurrentLimit;
  public int kDriveCurrentLimit;

  public double kMaxAngularSpeedRadPerSec;

  // the driving motor ports
  public int kFrontLeftDriveMotorPort;
  public int kFrontRightDriveMotorPort;
  public int kRearLeftDriveMotorPort;
  public int kRearRightDriveMotorPort;

  // whether the driving encoders are flipped
  public InvertedValue kFrontLeftDriveMotorReversed;
  public InvertedValue kRearLeftDriveMotorReversed;
  public InvertedValue kFrontRightDriveMotorReversed;
  public InvertedValue kRearRightDriveMotorReversed;

  // the turning motor ports
  public int kFrontLeftTurningMotorPort;
  public int kFrontRightTurningMotorPort;
  public int kRearLeftTurningMotorPort;
  public int kRearRightTurningMotorPort;

  // whether the turning enoders are flipped
  public InvertedValue kFrontLeftTurningMotorReversed;
  public InvertedValue kFrontRightTurningMotorReversed;
  public InvertedValue kRearLeftTurningMotorReversed;
  public InvertedValue kRearRightTurningMotorReversed;

  // the CANCoder turning encoder ports
  public int kFrontLeftCANcoderPort;
  public int kFrontRightCANcoderPort;
  public int kRearLeftCANcoderPort;
  public int kRearRightCANcoderPort;

  // whether the turning CANCoders are flipped
  public SensorDirectionValue kFrontLeftCANcoderDirection;
  public SensorDirectionValue kFrontRightCANcoderDirection;
  public SensorDirectionValue kRearLeftCANcoderDirection;
  public SensorDirectionValue kRearRightCANcoderDirection;

  // magnetic offset for the CANCoders
  public double kFrontLeftCANcoderMagnetOffset;
  public double kFrontRightCANcoderMagnetOffset;
  public double kRearLeftCANcoderMagnetOffset;
  public double kRearRightCANcoderMagnetOffset;

  // stats used by SwerveSubsystem for math
  public double kWheelDiameterMeters;
  public double kDriveBaseWidth;
  public double kDriveBaseLength;

  // stats used by SwerveSubsystem for deadbanding
  public double kXSpeedDeadband;
  public double kXSpeedMinValue;
  public double kYSpeedDeadband;
  public double kYSpeedMinValue;
  public double kRotDeadband;
  public double kRotMinValue;

  public boolean kFieldRelative;
  public boolean kOptimizeAngles;
  public boolean kPIDDirection;
  public double kDirectionP;
  public double kDirectionI;
  public double kDirectionD;
  public double kDirectionMultiplier;

  // PID values for the driving
  public double kDriveP;
  public double kDriveI;
  public double kDriveD;
  public double kDriveIZ;
  public double kDriveFF;
  public double kDriveV;
  public double kDriveMinOutput;
  public double kDriveMaxOutput;

  // multiplies the output speed of all of the drive motors, ALWAYS (0, 1).
  public double kDefaultSpeedMultiplier;
  public double kRotationSpeedMultiplier;
  public double kIntakeSpeedMultiplier;
  public double kAutonSpeedMultiplier;

  public double kDriveMaxRPM;
  public double kDriveStatorLimit;
  public double kDriveSupplyLimit;

  // PID values for the turning
  public double kTurnP;
  public double kTurnI;
  public double kTurnD;
  public double kTurnIZ;
  public double kTurnFF;
  public double kTurnMinOutput;
  public double kTurnMaxOutput;
  public int kTurnStatorLimit;
  public double kTurnSupplyLimit;

  public double kHeadingP;
  public double kHeadingI;
  public double kHeadingD;

  // because the turn gearing ratio is not 1:1, we need to spin the motor many
  // times to equal one spin of the module
  // this constant is used for the position conversion factor. (every 150 turns of
  // motors is 7 rotations of the module)
  public double kTurnConversionFactor;

  // because the drive gearing ratio is not 1:1, we need to spin the motor many
  // times to equal one spin of the module
  public double kDriveGearRatio;
  public double kThursdayHackGearRatio;
  public double kThursdayHackDirection;

  // Motion Magic configuration for drive motors (velocity control with trapezoid
  // profiling)
  public double kDriveMotionMagicAcceleration;
  public double kDriveMotionMagicJerk;

  // Motion Magic configuration for turn motors (position control with trapezoid
  // profiling)
  public double kTurnMotionMagicCruiseVelocity;
  public double kTurnMotionMagicAcceleration;
  public double kTurnMotionMagicJerk;

  public int kPigeonCANId;

  /** ABot (TalonFX) constants. */
  public static final SwerveConstants ABOT = fromTalonFX();

  /** BBot (Spark) constants. */
  public static final SwerveConstants BBOT = fromSpark();

  /** Active constants for the currently-selected robot variant. */
  public static final SwerveConstants INSTANCE = BotConstants.robotType == RobotVariant.BBOT ? BBOT : ABOT;

  /** Alias for readability in call sites. */
  public static final SwerveConstants ACTIVE = INSTANCE;

  private SwerveConstants() {
  }

  private static SwerveConstants fromSpark() {
    final SwerveConstants c = new SwerveConstants();

    c.rearLeftTranslation = SwerveConstantsSpark.rearLeftTranslation;
    c.rearRightTranslation = SwerveConstantsSpark.rearRightTranslation;
    c.frontRightTranslation = SwerveConstantsSpark.frontRightTranslation;
    c.frontLeftTranslation = SwerveConstantsSpark.frontLeftTranslation;

    c.headingControl = SwerveConstantsSpark.headingControl;

    c.kMaxSpeedMPSNormElevator = SwerveConstantsSpark.kMaxSpeedMPSNormElevator;
    c.kMaxSpeedMPSTopElevator = SwerveConstantsSpark.kMaxSpeedMPSTopElevator;
    c.tempMaxSpeed = SwerveConstantsSpark.tempMaxSpeed;
    c.kTurnCurrentLimit = SwerveConstantsSpark.kTurnCurrentLimit;
    c.kDriveCurrentLimit = SwerveConstantsSpark.kDriveCurrentLimit;
    c.kMaxAngularSpeedRadPerSec = SwerveConstantsSpark.kMaxAngularSpeedRadPerSec;

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

    c.kWheelDiameterMeters = SwerveConstantsSpark.kWheelDiameterMeters;
    c.kDriveBaseWidth = SwerveConstantsSpark.kDriveBaseWidth;
    c.kDriveBaseLength = SwerveConstantsSpark.kDriveBaseLength;

    c.kXSpeedDeadband = SwerveConstantsSpark.kXSpeedDeadband;
    c.kXSpeedMinValue = SwerveConstantsSpark.kXSpeedMinValue;
    c.kYSpeedDeadband = SwerveConstantsSpark.kYSpeedDeadband;
    c.kYSpeedMinValue = SwerveConstantsSpark.kYSpeedMinValue;
    c.kRotDeadband = SwerveConstantsSpark.kRotDeadband;
    c.kRotMinValue = SwerveConstantsSpark.kRotMinValue;

    c.kFieldRelative = SwerveConstantsSpark.kFieldRelative;
    c.kOptimizeAngles = SwerveConstantsSpark.kOptimizeAngles;
    c.kPIDDirection = SwerveConstantsSpark.kPIDDirection;
    c.kDirectionP = SwerveConstantsSpark.kDirectionP;
    c.kDirectionI = SwerveConstantsSpark.kDirectionI;
    c.kDirectionD = SwerveConstantsSpark.kDirectionD;
    c.kDirectionMultiplier = SwerveConstantsSpark.kDirectionMultiplier;

    c.kDriveP = SwerveConstantsSpark.kDriveP;
    c.kDriveI = SwerveConstantsSpark.kDriveI;
    c.kDriveD = SwerveConstantsSpark.kDriveD;
    c.kDriveIZ = SwerveConstantsSpark.kDriveIZ;
    c.kDriveFF = SwerveConstantsSpark.kDriveFF;
    c.kDriveV = SwerveConstantsSpark.kDriveV;
    c.kDriveMinOutput = SwerveConstantsSpark.kDriveMinOutput;
    c.kDriveMaxOutput = SwerveConstantsSpark.kDriveMaxOutput;

    c.kDefaultSpeedMultiplier = SwerveConstantsSpark.kDefaultSpeedMultiplier;
    c.kRotationSpeedMultiplier = SwerveConstantsSpark.kRotationSpeedMultiplier;
    c.kIntakeSpeedMultiplier = SwerveConstantsSpark.kIntakeSpeedMultiplier;
    c.kAutonSpeedMultiplier = SwerveConstantsSpark.kAutonSpeedMultiplier;

    c.kDriveMaxRPM = SwerveConstantsSpark.kDriveMaxRPM;
    c.kDriveStatorLimit = SwerveConstantsSpark.kDriveStatorLimit;
    c.kDriveSupplyLimit = SwerveConstantsSpark.kDriveSupplyLimit;

    c.kTurnP = SwerveConstantsSpark.kTurnP;
    c.kTurnI = SwerveConstantsSpark.kTurnI;
    c.kTurnD = SwerveConstantsSpark.kTurnD;
    c.kTurnIZ = SwerveConstantsSpark.kTurnIZ;
    c.kTurnFF = SwerveConstantsSpark.kTurnFF;
    c.kTurnMinOutput = SwerveConstantsSpark.kTurnMinOutput;
    c.kTurnMaxOutput = SwerveConstantsSpark.kTurnMaxOutput;
    c.kTurnStatorLimit = SwerveConstantsSpark.kTurnStatorLimit;
    c.kTurnSupplyLimit = SwerveConstantsSpark.kTurnSupplyLimit;

    c.kHeadingP = SwerveConstantsSpark.kHeadingP;
    c.kHeadingI = SwerveConstantsSpark.kHeadingI;
    c.kHeadingD = SwerveConstantsSpark.kHeadingD;

    c.kTurnConversionFactor = SwerveConstantsSpark.kTurnConversionFactor;

    c.kDriveGearRatio = SwerveConstantsSpark.kDriveGearRatio;
    c.kThursdayHackGearRatio = SwerveConstantsSpark.kThursdayHackGearRatio;
    c.kThursdayHackDirection = SwerveConstantsSpark.kThursdayHackDirection;

    c.kDriveMotionMagicAcceleration = SwerveConstantsSpark.kDriveMotionMagicAcceleration;
    c.kDriveMotionMagicJerk = SwerveConstantsSpark.kDriveMotionMagicJerk;

    c.kTurnMotionMagicCruiseVelocity = SwerveConstantsSpark.kTurnMotionMagicCruiseVelocity;
    c.kTurnMotionMagicAcceleration = SwerveConstantsSpark.kTurnMotionMagicAcceleration;
    c.kTurnMotionMagicJerk = SwerveConstantsSpark.kTurnMotionMagicJerk;

    c.kPigeonCANId = SwerveConstantsSpark.kPigeonCANId;

    return c;
  }

  private static SwerveConstants fromTalonFX() {
    final SwerveConstants c = new SwerveConstants();

    c.rearLeftTranslation = SwerveConstantsTalonFX.rearLeftTranslation;
    c.rearRightTranslation = SwerveConstantsTalonFX.rearRightTranslation;
    c.frontRightTranslation = SwerveConstantsTalonFX.frontRightTranslation;
    c.frontLeftTranslation = SwerveConstantsTalonFX.frontLeftTranslation;

    c.headingControl = SwerveConstantsTalonFX.headingControl;

    c.kMaxSpeedMPSNormElevator = SwerveConstantsTalonFX.kMaxSpeedMPSNormElevator;
    c.kMaxSpeedMPSTopElevator = SwerveConstantsTalonFX.kMaxSpeedMPSTopElevator;
    c.tempMaxSpeed = SwerveConstantsTalonFX.tempMaxSpeed;
    c.kTurnCurrentLimit = SwerveConstantsTalonFX.kTurnCurrentLimit;
    c.kDriveCurrentLimit = SwerveConstantsTalonFX.kDriveCurrentLimit;
    c.kMaxAngularSpeedRadPerSec = SwerveConstantsTalonFX.kMaxAngularSpeedRadPerSec;

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

    c.kWheelDiameterMeters = SwerveConstantsTalonFX.kWheelDiameterMeters;
    c.kDriveBaseWidth = SwerveConstantsTalonFX.kDriveBaseWidth;
    c.kDriveBaseLength = SwerveConstantsTalonFX.kDriveBaseLength;

    c.kXSpeedDeadband = SwerveConstantsTalonFX.kXSpeedDeadband;
    c.kXSpeedMinValue = SwerveConstantsTalonFX.kXSpeedMinValue;
    c.kYSpeedDeadband = SwerveConstantsTalonFX.kYSpeedDeadband;
    c.kYSpeedMinValue = SwerveConstantsTalonFX.kYSpeedMinValue;
    c.kRotDeadband = SwerveConstantsTalonFX.kRotDeadband;
    c.kRotMinValue = SwerveConstantsTalonFX.kRotMinValue;

    c.kFieldRelative = SwerveConstantsTalonFX.kFieldRelative;
    c.kOptimizeAngles = SwerveConstantsTalonFX.kOptimizeAngles;
    c.kPIDDirection = SwerveConstantsTalonFX.kPIDDirection;
    c.kDirectionP = SwerveConstantsTalonFX.kDirectionP;
    c.kDirectionI = SwerveConstantsTalonFX.kDirectionI;
    c.kDirectionD = SwerveConstantsTalonFX.kDirectionD;
    c.kDirectionMultiplier = SwerveConstantsTalonFX.kDirectionMultiplier;

    c.kDriveP = SwerveConstantsTalonFX.kDriveP;
    c.kDriveI = SwerveConstantsTalonFX.kDriveI;
    c.kDriveD = SwerveConstantsTalonFX.kDriveD;
    c.kDriveIZ = SwerveConstantsTalonFX.kDriveIZ;
    c.kDriveFF = SwerveConstantsTalonFX.kDriveFF;
    c.kDriveV = SwerveConstantsTalonFX.kDriveV;
    c.kDriveMinOutput = SwerveConstantsTalonFX.kDriveMinOutput;
    c.kDriveMaxOutput = SwerveConstantsTalonFX.kDriveMaxOutput;

    c.kDefaultSpeedMultiplier = SwerveConstantsTalonFX.kDefaultSpeedMultiplier;
    c.kRotationSpeedMultiplier = SwerveConstantsTalonFX.kRotationSpeedMultiplier;
    c.kIntakeSpeedMultiplier = SwerveConstantsTalonFX.kIntakeSpeedMultiplier;
    c.kAutonSpeedMultiplier = SwerveConstantsTalonFX.kAutonSpeedMultiplier;

    c.kDriveMaxRPM = SwerveConstantsTalonFX.kDriveMaxRPM;
    c.kDriveStatorLimit = SwerveConstantsTalonFX.kDriveStatorLimit;
    c.kDriveSupplyLimit = SwerveConstantsTalonFX.kDriveSupplyLimit;

    c.kTurnP = SwerveConstantsTalonFX.kTurnP;
    c.kTurnI = SwerveConstantsTalonFX.kTurnI;
    c.kTurnD = SwerveConstantsTalonFX.kTurnD;
    c.kTurnIZ = SwerveConstantsTalonFX.kTurnIZ;
    c.kTurnFF = SwerveConstantsTalonFX.kTurnFF;
    c.kTurnMinOutput = SwerveConstantsTalonFX.kTurnMinOutput;
    c.kTurnMaxOutput = SwerveConstantsTalonFX.kTurnMaxOutput;
    c.kTurnStatorLimit = SwerveConstantsTalonFX.kTurnStatorLimit;
    c.kTurnSupplyLimit = SwerveConstantsTalonFX.kTurnSupplyLimit;

    c.kHeadingP = SwerveConstantsTalonFX.kHeadingP;
    c.kHeadingI = SwerveConstantsTalonFX.kHeadingI;
    c.kHeadingD = SwerveConstantsTalonFX.kHeadingD;

    c.kTurnConversionFactor = SwerveConstantsTalonFX.kTurnConversionFactor;

    c.kDriveGearRatio = SwerveConstantsTalonFX.kDriveGearRatio;
    c.kThursdayHackGearRatio = SwerveConstantsTalonFX.kThursdayHackGearRatio;
    c.kThursdayHackDirection = SwerveConstantsTalonFX.kThursdayHackDirection;

    c.kDriveMotionMagicAcceleration = SwerveConstantsTalonFX.kDriveMotionMagicAcceleration;
    c.kDriveMotionMagicJerk = SwerveConstantsTalonFX.kDriveMotionMagicJerk;

    c.kTurnMotionMagicCruiseVelocity = SwerveConstantsTalonFX.kTurnMotionMagicCruiseVelocity;
    c.kTurnMotionMagicAcceleration = SwerveConstantsTalonFX.kTurnMotionMagicAcceleration;
    c.kTurnMotionMagicJerk = SwerveConstantsTalonFX.kTurnMotionMagicJerk;

    c.kPigeonCANId = SwerveConstantsTalonFX.kPigeonCANId;

    return c;
  }
}
