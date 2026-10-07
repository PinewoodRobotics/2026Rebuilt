package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.BotConstants;
import frc.robot.util.AimPoint;

public class GlobalPosition extends SubsystemBase {
  private static GlobalPosition self;

  // TODO(localization): stub after Autobahn removal
  private static final Pose2d kFieldCenterPose = new Pose2d(
      BotConstants.kFieldLayout.getFieldLength() / 2.0,
      BotConstants.kFieldLayout.getFieldWidth() / 2.0,
      new Rotation2d());

  public static enum GMFrame {
    kFieldRelative,
    kRobotRelative,
  }

  public static GlobalPosition GetInstance() {
    if (self == null) {
      self = new GlobalPosition();
    }
    return self;
  }

  // TODO(localization): stub after Autobahn removal
  public static Pose2d Get() {
    return kFieldCenterPose;
  }

  // TODO(localization): stub after Autobahn removal
  public static boolean isValid() {
    return false;
  }

  // TODO(localization): stub after Autobahn removal
  public static long getLastUpdateTimeMs() {
    return 0;
  }

  // TODO(localization): stub after Autobahn removal
  public static double[] getPositionCovariance() {
    return new double[0];
  }

  // TODO(localization): stub after Autobahn removal
  public static double[][] getPositionCovarianceMatrix() {
    return new double[0][0];
  }

  public static Translation2d Velocity2d(GMFrame velocityType) {
    var velocity = Velocity(velocityType);
    return new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
  }

  // TODO(localization): stub after Autobahn removal
  public static ChassisSpeeds Velocity(GMFrame velocityType) {
    return new ChassisSpeeds();
  }

  /**
   * Converts the velocity from the field frame to the robot frame.
   * 
   * @param rotationOfRobot The rotation of the robot in the field frame. This is
   *                        the angle of the robot in the field frame.
   * @return The velocity in the robot frame.
   */
  public static ChassisSpeeds VelocityInFrame(Rotation2d rotationOfRobot) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(Velocity(GMFrame.kFieldRelative), rotationOfRobot);
  }

  public static Pose2d ToRobotRelative(Pose2d pose) {
    return pose.relativeTo(Get());
  }

  public static Translation2d ToRobotRelative(Translation2d translation) {
    return translation.rotateBy(Get().getRotation());
  }

  @Override
  public void periodic() {
    Pose2d position = Get();

    Logger.recordOutput("Global/pose", position);
    Logger.recordOutput("Global/velocity", Velocity(GMFrame.kFieldRelative));

    for (AimPoint.ZoneName zoneName : AimPoint.ZoneName.values()) {
      AimPoint.logZoneForAdvantageScope(zoneName, "Global/Zones/All");
    }

    AimPoint.ZoneName activeZone = AimPoint.getZone(position);
    AimPoint.logZoneForAdvantageScope(activeZone, "Global/Zones/Active");

    var target = AimPoint.getTarget(activeZone);
    Logger.recordOutput(
        "Global/Zones/Active/LineToTarget",
        new Pose2d[] {
            new Pose2d(position.getTranslation(), new Rotation2d()),
            new Pose2d(target, new Rotation2d())
        });

    Logger.recordOutput("Global/Position/IsValid", isValid());
    Logger.recordOutput("Global/alliance", BotConstants.alliance);
  }
}
