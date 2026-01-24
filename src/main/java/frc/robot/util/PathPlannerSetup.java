package frc.robot.util;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystem.GlobalPosition;
import frc.robot.subsystem.OdometrySubsystem;
import frc.robot.subsystem.SwerveSubsystem;

public final class PathPlannerSetup {
  private static boolean configured = false;

  private PathPlannerSetup() {
  }

  public static void configure() {
    if (configured) {
      return;
    }

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      System.out.println("ERROR: PathPlanner RobotConfig load failed (GUI settings).");
      e.printStackTrace();
      return;
    }

    AutoBuilder.configure(
        new Supplier<Pose2d>() {

          @Override
          public Pose2d get() {
            return GlobalPosition.Get();
          }

        },
        OdometrySubsystem.GetInstance()::setOdometryPosition,
        PathPlannerSetup::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> SwerveSubsystem.GetInstance().drive(speeds, SwerveSubsystem.DriveType.RAW),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.2), // translation PID (initial: match ExecuteTrajectory)
            new PIDConstants(1.0, 0.0, 0.0) // rotation PID (initial: match ExecuteTrajectory theta P)
        ),
        config,
        PathPlannerSetup::shouldFlipForAlliance,
        SwerveSubsystem.GetInstance());

    configured = true;
  }

  private static boolean shouldFlipForAlliance() {
    // Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    // in the test field, we always start as red
    return false; // alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  /**
   * PathPlanner expects robot-relative speeds. Your
   * `GlobalPosition.GetVelocity()`
   * is field-relative, so convert using the current pose heading.
   */
  private static ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds field = GlobalPosition.GetVelocity();
    if (field == null) {
      return new ChassisSpeeds();
    }

    var heading = GlobalPosition.Get().getRotation();
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        field.vxMetersPerSecond,
        field.vyMetersPerSecond,
        field.omegaRadiansPerSecond,
        heading);
  }
}