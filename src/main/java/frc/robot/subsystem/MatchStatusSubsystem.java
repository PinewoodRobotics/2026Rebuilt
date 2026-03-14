package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.command.shooting.ContinuousShooter;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class MatchStatusSubsystem extends SubsystemBase {
  private static final String kDashboardRoot = "matchhud";
  private static final String kStateTableName = "state";

  private static final MatchStatusSubsystem instance = new MatchStatusSubsystem();

  private final MatchStatusCalculator calculator;

  private final IntegerPublisher seqPublisher;
  private final IntegerPublisher robotTimeUsPublisher;
  private final BooleanPublisher connectedPublisher;
  private final BooleanPublisher isRedAlliancePublisher;
  private final BooleanPublisher enabledPublisher;
  private final BooleanPublisher autonomousPublisher;
  private final StringPublisher gameSpecificMessagePublisher;
  private final StringPublisher fmsGameSpecificMessagePublisher;
  private final IntegerPublisher matchPhasePublisher;
  private final IntegerPublisher hubStatusPublisher;
  private final IntegerPublisher headerColorPublisher;
  private final DoublePublisher totalTimeRemainingSPublisher;
  private final DoublePublisher periodTimeRemainingSPublisher;
  private final DoublePublisher shiftTimeRemainingSPublisher;
  private final DoublePublisher shiftTimeWithBufferSPublisher;
  private final DoublePublisher bufferRemainingSPublisher;
  private final BooleanPublisher isShootingPublisher;
  private final BooleanPublisher showShiftIndicatorPublisher;
  private final BooleanPublisher showBufferPublisher;
  private final DoublePublisher robotPoseXMPublisher;
  private final DoublePublisher robotPoseYMPublisher;
  private final DoublePublisher robotHeadingRadPublisher;
  private final BooleanPublisher robotPoseValidPublisher;
  private final BooleanPublisher autoAlignActivePublisher;
  private final DoublePublisher autoAlignDistanceMPublisher;
  private final BooleanPublisher autoAlignReadyPublisher;
  private final BooleanPublisher driverOverridePublisher;
  private final IntegerPublisher aimModePublisher;
  private final StringPublisher cameraTopicPublisher;

  public static MatchStatusSubsystem GetInstance() {
    return instance;
  }

  MatchStatusSubsystem() {
    this(new MatchStatusCalculator(), NetworkTableInstance.getDefault());
  }

  MatchStatusSubsystem(MatchStatusCalculator calculator, NetworkTableInstance tableInstance) {
    this.calculator = calculator;

    NetworkTable stateTable = tableInstance.getTable(kDashboardRoot).getSubTable(kStateTableName);
    seqPublisher = stateTable.getIntegerTopic("seq").publish();
    robotTimeUsPublisher = stateTable.getIntegerTopic("robot_time_us").publish();
    connectedPublisher = stateTable.getBooleanTopic("connected").publish();
    isRedAlliancePublisher = stateTable.getBooleanTopic("is_red_alliance").publish();
    enabledPublisher = stateTable.getBooleanTopic("enabled").publish();
    autonomousPublisher = stateTable.getBooleanTopic("autonomous").publish();
    gameSpecificMessagePublisher = stateTable.getStringTopic("game_specific_message").publish();
    fmsGameSpecificMessagePublisher = tableInstance.getTable("FMSInfo").getStringTopic("GameSpecificMessage").publish();
    matchPhasePublisher = stateTable.getIntegerTopic("match_phase").publish();
    hubStatusPublisher = stateTable.getIntegerTopic("hub_status").publish();
    headerColorPublisher = stateTable.getIntegerTopic("header_color").publish();
    totalTimeRemainingSPublisher = stateTable.getDoubleTopic("total_time_remaining_s").publish();
    periodTimeRemainingSPublisher = stateTable.getDoubleTopic("period_time_remaining_s").publish();
    shiftTimeRemainingSPublisher = stateTable.getDoubleTopic("shift_time_remaining_s").publish();
    shiftTimeWithBufferSPublisher = stateTable.getDoubleTopic("shift_time_with_buffer_s").publish();
    bufferRemainingSPublisher = stateTable.getDoubleTopic("buffer_remaining_s").publish();
    isShootingPublisher = stateTable.getBooleanTopic("is_shooting").publish();
    showShiftIndicatorPublisher = stateTable.getBooleanTopic("show_shift_indicator").publish();
    showBufferPublisher = stateTable.getBooleanTopic("show_buffer").publish();
    robotPoseXMPublisher = stateTable.getDoubleTopic("robot_pose_x_m").publish();
    robotPoseYMPublisher = stateTable.getDoubleTopic("robot_pose_y_m").publish();
    robotHeadingRadPublisher = stateTable.getDoubleTopic("robot_heading_rad").publish();
    robotPoseValidPublisher = stateTable.getBooleanTopic("robot_pose_valid").publish();
    autoAlignActivePublisher = stateTable.getBooleanTopic("auto_align_active").publish();
    autoAlignDistanceMPublisher = stateTable.getDoubleTopic("auto_align_distance_m").publish();
    autoAlignReadyPublisher = stateTable.getBooleanTopic("auto_align_ready").publish();
    driverOverridePublisher = stateTable.getBooleanTopic("driver_override").publish();
    aimModePublisher = stateTable.getIntegerTopic("aim_mode").publish();
    cameraTopicPublisher = stateTable.getStringTopic("camera_topic").publish();
  }

  @Override
  public void periodic() {
    MatchStatusCalculator.State state = calculator.update(new MatchStatusCalculator.Inputs(
        RobotController.getTime(),
        System.currentTimeMillis(),
        DriverStation.isDSAttached(),
        Robot.getCommunicationClient().isConnected(),
        DriverStation.isEnabled(),
        DriverStation.isAutonomousEnabled(),
        DriverStation.isTeleopEnabled(),
        RobotContainer.isShooterArmedForHud(),
        ShooterSubsystem.getIsGpsAssistEnabled(),
        DriverStation.getAlliance(),
        DriverStation.getGameSpecificMessage(),
        GlobalPosition.Get(),
        GlobalPosition.getLastUpdateTimeMs()));

    seqPublisher.set(state.seq());
    robotTimeUsPublisher.set(state.robotTimeUs());
    connectedPublisher.set(state.connected());
    isRedAlliancePublisher.set(state.isRedAlliance());
    enabledPublisher.set(state.enabled());
    autonomousPublisher.set(state.autonomous());
    gameSpecificMessagePublisher.set(state.gameSpecificMessage());
    fmsGameSpecificMessagePublisher.set(state.gameSpecificMessage());
    matchPhasePublisher.set(state.matchPhase().ntValue());
    hubStatusPublisher.set(state.hubStatus().ntValue());
    headerColorPublisher.set(state.headerColor().ntValue());
    totalTimeRemainingSPublisher.set(state.totalTimeRemainingS());
    periodTimeRemainingSPublisher.set(state.periodTimeRemainingS());
    shiftTimeRemainingSPublisher.set(state.shiftTimeRemainingS());
    shiftTimeWithBufferSPublisher.set(state.shiftTimeWithBufferS());
    bufferRemainingSPublisher.set(state.bufferRemainingS());
    isShootingPublisher.set(ContinuousShooter.isShooting());
    showShiftIndicatorPublisher.set(state.showShiftIndicator());
    showBufferPublisher.set(state.showBuffer());
    robotPoseXMPublisher.set(state.robotPoseXM());
    robotPoseYMPublisher.set(state.robotPoseYM());
    robotHeadingRadPublisher.set(state.robotHeadingRad());
    robotPoseValidPublisher.set(state.robotPoseValid());
    autoAlignActivePublisher.set(state.autoAlignActive());
    autoAlignDistanceMPublisher.set(state.autoAlignDistanceM());
    autoAlignReadyPublisher.set(state.autoAlignReady());
    driverOverridePublisher.set(state.driverOverride());
    aimModePublisher.set(state.aimMode().ntValue());
    cameraTopicPublisher.set(state.cameraTopic());

    Logger.recordOutput("MatchStatus/seq", (double) state.seq());
    Logger.recordOutput("MatchStatus/robotTimeUs", (double) state.robotTimeUs());
    Logger.recordOutput("MatchStatus/connected", state.connected());
    Logger.recordOutput("MatchStatus/isRedAlliance", state.isRedAlliance());
    Logger.recordOutput("MatchStatus/enabled", state.enabled());
    Logger.recordOutput("MatchStatus/autonomous", state.autonomous());
    Logger.recordOutput("MatchStatus/gameSpecificMessage", state.gameSpecificMessage());
    Logger.recordOutput("MatchStatus/matchPhase", (int) state.matchPhase().ntValue());
    Logger.recordOutput("MatchStatus/hubStatus", (int) state.hubStatus().ntValue());
    Logger.recordOutput("MatchStatus/headerColor", (int) state.headerColor().ntValue());
    Logger.recordOutput("MatchStatus/totalTimeRemainingS", state.totalTimeRemainingS());
    Logger.recordOutput("MatchStatus/periodTimeRemainingS", state.periodTimeRemainingS());
    Logger.recordOutput("MatchStatus/shiftTimeRemainingS", state.shiftTimeRemainingS());
    Logger.recordOutput("MatchStatus/shiftTimeWithBufferS", state.shiftTimeWithBufferS());
    Logger.recordOutput("MatchStatus/bufferRemainingS", state.bufferRemainingS());
    Logger.recordOutput("MatchStatus/isShooting", ContinuousShooter.isShooting());
    Logger.recordOutput("MatchStatus/showShiftIndicator", state.showShiftIndicator());
    Logger.recordOutput("MatchStatus/showBuffer", state.showBuffer());
    Logger.recordOutput("MatchStatus/robotPoseXM", state.robotPoseXM());
    Logger.recordOutput("MatchStatus/robotPoseYM", state.robotPoseYM());
    Logger.recordOutput("MatchStatus/robotHeadingRad", state.robotHeadingRad());
    Logger.recordOutput("MatchStatus/robotPoseValid", state.robotPoseValid());
    Logger.recordOutput("MatchStatus/autoAlignActive", state.autoAlignActive());
    Logger.recordOutput("MatchStatus/autoAlignDistanceM", state.autoAlignDistanceM());
    Logger.recordOutput("MatchStatus/autoAlignReady", state.autoAlignReady());
    Logger.recordOutput("MatchStatus/driverOverride", state.driverOverride());
    Logger.recordOutput("MatchStatus/aimMode", (int) state.aimMode().ntValue());
    Logger.recordOutput("MatchStatus/cameraTopic", state.cameraTopic());
  }
}
