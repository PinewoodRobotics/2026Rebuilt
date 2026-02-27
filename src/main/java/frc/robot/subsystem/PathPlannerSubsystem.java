package frc.robot.subsystem;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.google.protobuf.InvalidProtocolBufferException;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PathPlannerLogging;

import autobahn.client.NamedCallback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constant.PathPlannerConstants;
import frc.robot.subsystem.GlobalPosition.GMFrame;
import frc4765.proto.util.Other.SelectedPath;

public final class PathPlannerSubsystem extends SubsystemBase {
  private final RobotConfig robotConfig;
  private volatile Pose2d[] activePath = new Pose2d[0];
  private volatile PathPlannerPath activePathObject;

  private static PathPlannerSubsystem self;

  public static PathPlannerSubsystem GetInstance() {
    if (self == null) {
      self = new PathPlannerSubsystem();
    }

    return self;
  }

  public PathPlannerSubsystem() {
    Pathfinding.setPathfinder(new LocalADStar());
    PathfindingCommand.warmupCommand();

    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      throw new RuntimeException("Failed to load RobotConfig", e);
    }

    AutoBuilder.configure(
        () -> GlobalPosition.Get(),
        OdometrySubsystem.GetInstance()::setOdometryPosition,
        () -> GlobalPosition.Velocity(GMFrame.kRobotRelative),
        (speeds, feedforwards) -> SwerveSubsystem.GetInstance().drive(speeds, SwerveSubsystem.DriveType.RAW),
        PathPlannerConstants.defaultPathfindingController,
        robotConfig,
        PathPlannerSubsystem::shouldFlipForAlliance,
        SwerveSubsystem.GetInstance());

    PathPlannerLogging.setLogActivePathCallback(path -> {
      activePath = path.toArray(Pose2d[]::new);
    });

    Robot.getCommunicationClient().subscribe(PathPlannerConstants.kPathSelectedTopic,
        NamedCallback.FromConsumer(this::subscription));
  }

  public Command getAutoCommand() {
    if (GlobalPosition.Get() == null || activePathObject == null) {
      return Commands.none();
    }

    var startPose = activePathObject.getPathPoses().get(0);
    if (GlobalPosition.Get().getTranslation().getDistance(startPose.getTranslation()) < 1) {
      return AutoBuilder.followPath(activePathObject);
    }

    return AutoBuilder.pathfindToPose(activePathObject.getPathPoses().get(0),
        PathPlannerConstants.defaultPathfindingConstraints, 0);
  }

  private Pose2d[] pathToPose2dArray(PathPlannerPath path) {
    return path.getPathPoses().toArray(new Pose2d[0]);
  }

  private void subscription(byte[] payload) {
    SelectedPath selectedPath;
    try {
      selectedPath = SelectedPath.parseFrom(payload);
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
      return;
    }

    String pathName = selectedPath.getPathName();

    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
      return;
    }

    activePath = pathToPose2dArray(path);
    activePathObject = path;
  }

  private static boolean shouldFlipForAlliance() {
    // TODO: Make this dynamic based on the alliance color.
    // In the test field, we always start as red.
    return false;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("PathPlanner/FollowingPath", activePath.length > 0);
    Logger.recordOutput("PathPlanner/CurrentPath", activePath);
    Logger.recordOutput("PathPlanner/CurrentPathPointCount", activePath.length);
  }
}
