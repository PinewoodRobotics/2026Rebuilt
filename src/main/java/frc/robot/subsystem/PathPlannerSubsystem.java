package frc.robot.subsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.units.Units;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.robot.constant.BotConstants;
import frc.robot.constant.PathPlannerConstants;
import frc.robot.constant.PathPlannerConstants.SelectedAuto;
import frc.robot.subsystem.GlobalPosition.GMFrame;
import frc.robot.util.PathedAuto;
import org.littletonrobotics.junction.Logger;

public final class PathPlannerSubsystem extends SubsystemBase {
  private final RobotConfig robotConfig;
  private volatile SelectedAuto selectedAuto;

  private static PathPlannerSubsystem self;

  public Command currentAutoCommand;

  private String[] allAutos;

  public static PathPlannerSubsystem GetInstance() {
    if (self == null) {
      self = new PathPlannerSubsystem();
    }

    return self;
  }

  public PathPlannerSubsystem() {
    Pathfinding.setPathfinder(new LocalADStar());
    // CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    this.robotConfig = loadRobotConfig();
    configureAutoBuilder();

    this.selectedAuto = new SelectedAuto(PathPlannerSubsystem::shouldFlipForAlliance);
    this.allAutos = AutoBuilder.getAllAutoNames().toArray(new String[0]);
  }

  private RobotConfig loadRobotConfig() {
    try {
      return RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load RobotConfig from GUI settings", e);
    }
  }

  private void configureAutoBuilder() {
    AutoBuilder.configure(
        () -> GlobalPosition.Get(),
        (PathPlannerSubsystem::resetOdom),
        () -> GlobalPosition.Velocity(GMFrame.kRobotRelative),
        (speeds, feedforwards) -> SwerveSubsystem.GetInstance().drive(speeds, SwerveSubsystem.DriveType.RAW),
        PathPlannerConstants.defaultPathfindingController,
        robotConfig,
        PathPlannerSubsystem::shouldFlipForAlliance,
        SwerveSubsystem.GetInstance());
  }

  private static void resetOdom(Pose2d pose) {
    OdometrySubsystem.GetInstance().resetPose(pose);
  }

  public Command getAutoCommand() {
    if (!isSelectedAutoValid()) {
      return Commands.none();
    }

    return selectedAuto.getCurrentAuto().get();
  }

  public boolean isSelectedAutoValid() {
    return selectedAuto.getCurrentAuto().isPresent();
  }

  public Command getAndInitAutoCommand(boolean pathfindIfNotAtStart) {
    if (!isSelectedAutoValid()) {
      return Commands.none();
    }

    PathedAuto auto = selectedAuto.getCurrentAuto().get();
    currentAutoCommand = auto;

    Pose2d startingPose = auto.getStartingPose();
    if (!pathfindIfNotAtStart && startingPose != null) {
      resetOdom(shouldFlipForAlliance() ? FlippingUtil.flipFieldPose(startingPose) : startingPose);
    }

    Pose2d[] pathPoses = selectedAuto.getPathPoses(0);
    if (pathfindIfNotAtStart && pathPoses.length > 0 && pathPoses[0].getTranslation()
        .getDistance(GlobalPosition.Get().getTranslation()) > PathPlannerConstants.distanceConsideredOffTarget
            .in(Units.Meters)) {
      currentAutoCommand = AutoBuilder.pathfindToPose(pathPoses[0],
          PathPlannerConstants.defaultPathfindingConstraints);
    }

    return currentAutoCommand;
  }

  private static boolean shouldFlipForAlliance() {
    return BotConstants.alliance != Alliance.RED;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("PathPlanner/CurrentPath", selectedAuto.getAllPathPoses());
    Logger.recordOutput("PathPlanner/CurrentSelectedAuto", selectedAuto.getName());
    Logger.recordOutput("PathPlanner/SelectedAutoValid", selectedAuto.getCurrentAuto().isPresent());
    Logger.recordOutput("PathPlanner/ValidNames/Autos", allAutos);
  }
}
