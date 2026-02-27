package frc.robot.constant;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class PathPlannerConstants {
  public static final PPHolonomicDriveController defaultPathfindingController = new PPHolonomicDriveController(
      new PIDConstants(3.0, 0.0, 0.1),
      new PIDConstants(0.5, 0.0, 0.3));
  public static final PathConstraints defaultPathfindingConstraints = new PathConstraints(1.0, 1.0,
      Units.degreesToRadians(260), Units.degreesToRadians(260));

  public static final String kPathSelectedTopic = "pathplanner/path";

  public static final Pose2d kBallShooterRightStartPose = new Pose2d(14.459, 4.039, Rotation2d.fromDegrees(125));
}
