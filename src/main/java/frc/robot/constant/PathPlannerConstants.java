package frc.robot.constant;

import java.util.EnumSet;
import java.util.Optional;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.command.SwerveMoveTeleop.AxisConstraint;
import frc.robot.command.SwerveMoveTeleop.Lane;
import frc.robot.util.PathedAuto;
import frc.robot.util.SharedStringTopic;
import lombok.Getter;

public class PathPlannerConstants {
  /**
   * Holds the currently selected autonomous routine and its path.
   * Subscribes to the auto selection NetworkTables topic and updates
   * the auto, path, and path poses when the selection changes.
   */
  public static class SelectedAuto {
    private static final String kNoneSelection = "NONE";

    private String name;
    private Optional<PathedAuto> currentAuto;

    private final boolean shouldFlip;
    private final SharedStringTopic kAutoSelect;

    public SelectedAuto(boolean shouldFlip) {
      this(shouldFlip, kAutoSelectTopic);
    }

    /**
     * Creates a SelectedAuto that listens for auto selection changes.
     *
     * @param shouldFlip whether to flip the auto for the opposite alliance
     */
    public SelectedAuto(boolean shouldFlip, String topicBase) {
      this.kAutoSelect = new SharedStringTopic(topicBase);
      this.shouldFlip = shouldFlip;
      this.name = kNoneSelection;
      this.currentAuto = Optional.empty();

      kAutoSelect.setState(this.name);
    }

    private void clearSelection() {
      name = kNoneSelection;
      currentAuto = Optional.empty();
      kAutoSelect.setState(name);
    }

    private void updateFromSelection(String selected) {
      if (selected.equals(name))
        return;

      if (selected == null || selected.isEmpty() || kNoneSelection.equalsIgnoreCase(selected)) {
        clearSelection();
        return;
      }

      name = selected;
      currentAuto = Optional.of(new PathedAuto(name, shouldFlip));
      kAutoSelect.setState(name);
    }

    private void updateFromSelection() {
      updateFromSelection(kAutoSelect.getState());
    }

    public Pose2d[] getPathPoses(int index) {
      updateFromSelection();

      if (!currentAuto.isPresent()) {
        return new Pose2d[0];
      }

      return currentAuto.get().getPaths().get(index).getPathPoses().toArray(new Pose2d[0]);
    }

    public Pose2d[] getAllPathPoses() {
      return getPathPoses(0);
    }

    public String getName() {
      updateFromSelection();
      return name;
    }

    public Optional<PathedAuto> getCurrentAuto() {
      updateFromSelection();
      return currentAuto;
    }
  }

  public static final PPHolonomicDriveController defaultPathfindingController = new PPHolonomicDriveController(
      new PIDConstants(3.0, 0.0, 0.1),
      new PIDConstants(0.5, 0.0, 0.3));

  public static final PathConstraints defaultPathfindingConstraints = new PathConstraints(1.0, 1.0,
      Units.degreesToRadians(360), Units.degreesToRadians(720));

  public static final Distance distanceConsideredOffTarget = edu.wpi.first.units.Units.Meters.of(1.0);

  public static final String kAutoSelectTopic = "PathPlanner/SelectedPath";

  public static final Lane[] kLanes = {
      new Lane(new Pose2d(11.94, 7.52, new Rotation2d(1, 0)), 1.0, AxisConstraint.Y),
  };
}
