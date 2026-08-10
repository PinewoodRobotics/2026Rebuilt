package frc.robot.command;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Endgame climb sequence from the PathPlanner Climber autos (after the robot is
 * already in
 * position): raise climber with intake wrist to top, follow the short
 * align-to-cage path, then
 * lower climber.
 *
 * <p>
 * Use {@link ClosePath#LEFT} for the same close path as
 * {@code Climber Right Right.auto}
 * ({@code Climber Left Close.path}), and {@link ClosePath#RIGHT} for
 * {@code Climber Right Left.auto}
 * ({@code Climber Right Close.path}).
 */
public class ClimbAuto extends SequentialCommandGroup {

  public enum ClosePath {
    /** Same as the close segment in {@code Climber Right Right.auto}. */
    LEFT("Climber Left Close"),
    /** Same as the close segment in {@code Climber Right Left.auto}. */
    RIGHT("Climber Right Close");

    public final String pathFileName;

    ClosePath(String pathFileName) {
      this.pathFileName = pathFileName;
    }
  }

  public ClimbAuto(ClosePath closePath) {
    addCommands(
        new ParallelDeadlineGroup(
            NamedCommands.getCommand("MoveClimberUp"),
            NamedCommands.getCommand("IntakeTopCommand")),
        FollowPath(closePath),
        NamedCommands.getCommand("MoveClimberDown"));
  }

  private static Command FollowPath(ClosePath closePath) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(closePath.pathFileName);
      return AutoBuilder.followPath(path);
    } catch (IOException | ParseException | FileVersionException e) {
      throw new RuntimeException("Failed to load PathPlanner path: " + closePath.pathFileName, e);
    }
  }
}
