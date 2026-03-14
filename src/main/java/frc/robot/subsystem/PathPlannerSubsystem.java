package frc.robot.subsystem;

import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.command.shooting.ContinuousShooter;
import frc.robot.constant.BotConstants;
import frc.robot.constant.CommunicationConstants;
import frc.robot.constant.PathPlannerConstants;
import frc.robot.constant.PathPlannerConstants.SelectedAuto;
import frc.robot.subsystem.GlobalPosition.GMFrame;
import frc.robot.util.PathedAuto;

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
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    this.robotConfig = loadRobotConfig();
    configureAutoBuilder();

    this.selectedAuto = new SelectedAuto(shouldFlipForAlliance());
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
        OdometrySubsystem.GetInstance()::setOdometryPosition,
        () -> GlobalPosition.Velocity(GMFrame.kRobotRelative),
        (speeds, feedforwards) -> SwerveSubsystem.GetInstance().drive(speeds, SwerveSubsystem.DriveType.RAW),
        PathPlannerConstants.defaultPathfindingController,
        robotConfig,
        PathPlannerSubsystem::shouldFlipForAlliance,
        SwerveSubsystem.GetInstance());
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

    currentAutoCommand = selectedAuto.getCurrentAuto().get();
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
    return DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("PathPlanner/CurrentPath", selectedAuto.getAllPathPoses());
    Logger.recordOutput("PathPlanner/CurrentSelectedAuto", selectedAuto.getName());
    Logger.recordOutput("PathPlanner/SelectedAutoValid", selectedAuto.getCurrentAuto().isPresent());
    Logger.recordOutput("PathPlanner/ValidNames/Autos", allAutos);
  }
}
