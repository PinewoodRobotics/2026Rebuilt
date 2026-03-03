package frc.robot.constant;

import java.util.EnumSet;
import java.util.Optional;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import frc.robot.Robot;
import frc.robot.util.PathedAuto;

public class PathPlannerConstants {
  /**
   * Holds the currently selected autonomous routine and its path.
   * Subscribes to the auto selection NetworkTables topic and updates
   * the auto, path, and path poses when the selection changes.
   */
  public static class SelectedAuto {
    private static final String kNoneSelection = "NONE";

    private String name;
    private Optional<PathedAuto> currentAuto = Optional.empty();

    private final boolean shouldFlip;
    private final StringSubscriber requestSubscriber;
    private final StringPublisher statePublisher;

    /**
     * Creates a SelectedAuto that listens for auto selection changes.
     *
     * @param shouldFlip whether to flip the auto for the opposite alliance
     */
    public SelectedAuto(boolean shouldFlip) {
      this.shouldFlip = shouldFlip;
      this.requestSubscriber = kAutoSelectRequestTopic.subscribe(kNoneSelection);
      this.statePublisher = kAutoSelectStateTopic.publish();
      clearSelection();
      statePublisher.set(kNoneSelection);

      Robot.getNetworkTableInstance().addListener(
          requestSubscriber,
          EnumSet.of(NetworkTableEvent.Kind.kValueAll),
          event -> {
            if (event.valueData == null || event.valueData.value == null) {
              return;
            }

            String value = event.valueData.value.getString();
            if (value == null) {
              return;
            }

            updateFromSelection(value);
          });
    }

    private void updateFromSelection(String selected) {
      if (selected == null || selected.isEmpty() || kNoneSelection.equalsIgnoreCase(selected)) {
        clearSelection();
        return;
      }

      name = selected;
      currentAuto = Optional.of(new PathedAuto(name, shouldFlip));
      statePublisher.set(name);
    }

    private void clearSelection() {
      name = kNoneSelection;
      currentAuto = Optional.empty();
      statePublisher.set(kNoneSelection);
    }

    public String getName() {
      return name;
    }

    public Pose2d[] getPathPoses(int index) {
      if (!currentAuto.isPresent()) {
        return new Pose2d[0];
      }

      return currentAuto.get().getPaths().get(index).getPathPoses().toArray(new Pose2d[0]);
    }

    public Pose2d[] getAllPathPoses() {
      return getPathPoses(0);
    }

    public Optional<PathedAuto> getCurrentAuto() {
      return currentAuto;
    }
  }

  public static final PPHolonomicDriveController defaultPathfindingController = new PPHolonomicDriveController(
      new PIDConstants(3.0, 0.0, 0.1),
      new PIDConstants(0.5, 0.0, 0.3));
  public static final PathConstraints defaultPathfindingConstraints = new PathConstraints(1.0, 1.0,
      Units.degreesToRadians(360), Units.degreesToRadians(720));
  public static final double distanceConsideredOffTarget = 1;

  public static final StringTopic kAutoSelectRequestTopic = CommunicationConstants.kDashboardTable
      .getStringTopic(CommunicationConstants.kAutoSelectRequestTopic);
  public static final StringTopic kAutoSelectStateTopic = CommunicationConstants.kDashboardTable
      .getStringTopic(CommunicationConstants.kAutoSelectStateTopic);
}
