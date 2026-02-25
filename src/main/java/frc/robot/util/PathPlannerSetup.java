package frc.robot.util;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.GlobalPosition;
import frc.robot.subsystem.GlobalPosition.GMFrame;
import frc.robot.subsystem.OdometrySubsystem;
import frc.robot.subsystem.SwerveSubsystem;

public final class PathPlannerSetup {
  private static boolean configured = false;
  private static RobotConfig robotConfig = null;

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
    robotConfig = config;

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
            new PIDConstants(3.0, 0.0, 0.1), // translation PID (initial: match ExecuteTrajectory)
            new PIDConstants(0.5, 0.0, 0.2) // rotation PID (initial: match ExecuteTrajectory theta P)
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
    return GlobalPosition.Velocity(GMFrame.kRobotRelative);
  }

  /**
   * Returns the trajectory for a path by name (as in the PathPlanner GUI).
   * Applies alliance flip when {@link #shouldFlipForAlliance()} is true.
   *
   * @param pathName name of the path file (e.g. "Ball Shooter Right")
   * @return the generated trajectory, or empty if not configured, path load
   *         failed, or ideal trajectory could not be generated
   */
  public static Optional<PathPlannerTrajectory> getTrajectory(String pathName) {
    if (!configured || robotConfig == null) {
      return Optional.empty();
    }
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      if (shouldFlipForAlliance()) {
        path = path.flipPath();
      }

      return path.getIdealTrajectory(robotConfig);
    } catch (Exception e) {
      System.out.println("ERROR: PathPlanner getTrajectory failed for path: " + pathName);
      e.printStackTrace();
      return Optional.empty();
    }
  }

  /**
   * Returns the autonomous command configured in PathPlanner (e.g. "Ball Shooter
   * Right").
   */
  public static Command getAutonomousCommand() {
    return AutoBuilder.buildAuto("Ball Shooter Right");
  }
}