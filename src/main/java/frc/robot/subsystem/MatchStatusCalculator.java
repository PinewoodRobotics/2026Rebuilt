package frc.robot.subsystem;

import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constant.CommunicationConstants;

public class MatchStatusCalculator {
  static final double kAutoDurationS = 20.0;
  static final double kTransitionDurationS = 10.0;
  static final double kShiftDurationS = 25.0;
  static final double kEndgameDurationS = 30.0;
  static final double kTeleopDurationS = kTransitionDurationS + (4.0 * kShiftDurationS) + kEndgameDurationS;
  static final double kFullMatchDurationS = kAutoDurationS + kTeleopDurationS;
  static final double kHubWarningDurationS = 3.0;
  static final long kPoseFreshnessThresholdMs = 250;

  private enum SessionType {
    NONE,
    FULL_MATCH,
    TELEOP_ONLY
  }

  public enum MatchPhase {
    PRE_MATCH(0),
    AUTONOMOUS(1),
    TRANSITION(2),
    SHIFT1(3),
    SHIFT2(4),
    SHIFT3(5),
    SHIFT4(6),
    ENDGAME(7),
    POST_MATCH(8);

    private final long ntValue;

    MatchPhase(long ntValue) {
      this.ntValue = ntValue;
    }

    public long ntValue() {
      return ntValue;
    }
  }

  public enum HubStatus {
    HUB_NONE(0),
    HUB_BOTH(1),
    HUB_ACTIVE(2),
    HUB_WARNING(3),
    HUB_INACTIVE(4);

    private final long ntValue;

    HubStatus(long ntValue) {
      this.ntValue = ntValue;
    }

    public long ntValue() {
      return ntValue;
    }
  }

  public enum HeaderColor {
    HEADER_HIDDEN(0),
    HEADER_GREEN(1),
    HEADER_YELLOW(2),
    HEADER_PURPLE(3);

    private final long ntValue;

    HeaderColor(long ntValue) {
      this.ntValue = ntValue;
    }

    public long ntValue() {
      return ntValue;
    }
  }

  public record Inputs(
      long robotTimeUs,
      long wallClockMs,
      boolean dsAttached,
      boolean autobahnConnected,
      boolean enabled,
      boolean autonomousEnabled,
      boolean teleopEnabled,
      Optional<Alliance> alliance,
      String gameSpecificMessage,
      Pose2d pose,
      long poseLastUpdateMs) {
    public Inputs {
      alliance = Objects.requireNonNull(alliance);
      gameSpecificMessage = Objects.requireNonNullElse(gameSpecificMessage, "");
    }
  }

  public record State(
      long seq,
      long robotTimeUs,
      boolean connected,
      boolean isRedAlliance,
      boolean enabled,
      boolean autonomous,
      String gameSpecificMessage,
      MatchPhase matchPhase,
      HubStatus hubStatus,
      HeaderColor headerColor,
      double totalTimeRemainingS,
      double periodTimeRemainingS,
      double shiftTimeRemainingS,
      double shiftTimeWithBufferS,
      double bufferRemainingS,
      boolean showShiftIndicator,
      boolean showBuffer,
      double robotPoseXM,
      double robotPoseYM,
      double robotHeadingRad,
      boolean robotPoseValid,
      boolean autoAlignActive,
      double autoAlignDistanceM,
      boolean autoAlignReady,
      boolean driverOverride,
      String cameraTopic) {
  }

  private long seq;
  private SessionType sessionType = SessionType.NONE;
  private long matchStartUs = -1;
  private long teleopStartUs = -1;
  private long preTeleopDisabledPauseUs;
  private long preTeleopDisabledPauseStartUs = -1;
  private MatchPhase lastActivePhase = MatchPhase.PRE_MATCH;
  private Pose2d lastValidPose;
  private boolean sessionArmedForReset;

  public State update(Inputs inputs) {
    if (sessionArmedForReset && inputs.enabled()) {
      resetSession();
    }

    startSessionIfNeeded(inputs);
    updatePreTeleopDisabledPause(inputs);

    long robotTimeUs = inputs.robotTimeUs();
    double matchElapsedS = getMatchElapsedSeconds(robotTimeUs);
    double teleopElapsedS = secondsSince(teleopStartUs, robotTimeUs);

    MatchPhase phase = resolvePhase(inputs, robotTimeUs, teleopElapsedS);
    if (inputs.enabled() && phase != MatchPhase.PRE_MATCH && phase != MatchPhase.POST_MATCH) {
      lastActivePhase = phase;
    }

    boolean isRedAlliance = inputs.alliance().orElse(Alliance.Blue) == Alliance.Red;
    String gameSpecificMessage = resolveGameSpecificMessage(inputs.gameSpecificMessage(), isRedAlliance);
    boolean ourHubInactiveInShift1 = doesGameDataDeactivateUsInShift1(gameSpecificMessage, isRedAlliance);

    double periodRemainingS = computePeriodTimeRemaining(phase, matchElapsedS, teleopElapsedS);
    double totalRemainingS = computeTotalTimeRemaining(matchElapsedS, teleopElapsedS);
    double shiftTimeRemainingS = computeShiftTimeRemaining(phase, teleopElapsedS);
    double shiftTimeWithBufferS = computeShiftTimeWithBuffer(phase, teleopElapsedS, ourHubInactiveInShift1);

    double phaseElapsedS = getPhaseElapsedSeconds(phase, teleopElapsedS);
    HubStatus hubStatus = resolveHubStatus(phase, phaseElapsedS, ourHubInactiveInShift1);
    boolean showBuffer = hubStatus == HubStatus.HUB_WARNING;
    double bufferRemainingS = showBuffer ? clamp(kHubWarningDurationS - phaseElapsedS) : 0.0;

    PoseSnapshot poseSnapshot = resolvePose(inputs);
    boolean driverOverride = false;

    State state = new State(
        seq++,
        robotTimeUs,
        inputs.dsAttached() && inputs.autobahnConnected(),
        isRedAlliance,
        inputs.enabled(),
        inputs.autonomousEnabled(),
        gameSpecificMessage,
        phase,
        hubStatus,
        resolveHeaderColor(phase, hubStatus, driverOverride),
        totalRemainingS,
        periodRemainingS,
        shiftTimeRemainingS,
        shiftTimeWithBufferS,
        bufferRemainingS,
        isShiftIndicatorVisible(phase),
        showBuffer,
        poseSnapshot.pose().getX(),
        poseSnapshot.pose().getY(),
        poseSnapshot.pose().getRotation().getRadians(),
        poseSnapshot.valid(),
        false,
        0.0,
        false,
        driverOverride,
        CommunicationConstants.kMatchStatusCameraTopic);

    if (phase == MatchPhase.POST_MATCH && !inputs.enabled()) {
      sessionArmedForReset = true;
    }

    return state;
  }

  private void startSessionIfNeeded(Inputs inputs) {
    if (sessionType != SessionType.NONE) {
      if (sessionType == SessionType.FULL_MATCH && teleopStartUs < 0 && inputs.teleopEnabled()) {
        teleopStartUs = inputs.robotTimeUs();
      }
      return;
    }

    if (inputs.autonomousEnabled()) {
      sessionType = SessionType.FULL_MATCH;
      matchStartUs = inputs.robotTimeUs();
    } else if (inputs.teleopEnabled()) {
      sessionType = SessionType.TELEOP_ONLY;
      teleopStartUs = inputs.robotTimeUs();
    }
  }

  private MatchPhase resolvePhase(Inputs inputs, long robotTimeUs, double teleopElapsedS) {
    if (sessionType == SessionType.NONE) {
      return MatchPhase.PRE_MATCH;
    }

    if (inputs.autonomousEnabled()) {
      return MatchPhase.AUTONOMOUS;
    }

    if (inputs.teleopEnabled() && teleopStartUs >= 0) {
      return resolveTeleopPhase(teleopElapsedS);
    }

    if (isSessionComplete(robotTimeUs)) {
      return MatchPhase.POST_MATCH;
    }

    return lastActivePhase;
  }

  private MatchPhase resolveTeleopPhase(double teleopElapsedS) {
    if (teleopElapsedS < kTransitionDurationS) {
      return MatchPhase.TRANSITION;
    }
    if (teleopElapsedS < kTransitionDurationS + kShiftDurationS) {
      return MatchPhase.SHIFT1;
    }
    if (teleopElapsedS < kTransitionDurationS + (2.0 * kShiftDurationS)) {
      return MatchPhase.SHIFT2;
    }
    if (teleopElapsedS < kTransitionDurationS + (3.0 * kShiftDurationS)) {
      return MatchPhase.SHIFT3;
    }
    if (teleopElapsedS < kTransitionDurationS + (4.0 * kShiftDurationS)) {
      return MatchPhase.SHIFT4;
    }
    if (teleopElapsedS < kTeleopDurationS) {
      return MatchPhase.ENDGAME;
    }
    return MatchPhase.POST_MATCH;
  }

  private boolean isSessionComplete(long robotTimeUs) {
    return switch (sessionType) {
      case FULL_MATCH -> teleopStartUs >= 0 && secondsSince(teleopStartUs, robotTimeUs) >= kTeleopDurationS;
      case TELEOP_ONLY -> teleopStartUs >= 0 && secondsSince(teleopStartUs, robotTimeUs) >= kTeleopDurationS;
      case NONE -> false;
    };
  }

  private double computeTotalTimeRemaining(double matchElapsedS, double teleopElapsedS) {
    return switch (sessionType) {
      case FULL_MATCH -> clamp(kFullMatchDurationS - matchElapsedS);
      case TELEOP_ONLY -> clamp(kTeleopDurationS - teleopElapsedS);
      case NONE -> 0.0;
    };
  }

  private void updatePreTeleopDisabledPause(Inputs inputs) {
    boolean isPreTeleopDisabledGap = sessionType == SessionType.FULL_MATCH
        && teleopStartUs < 0
        && !inputs.enabled()
        && !inputs.autonomousEnabled()
        && !inputs.teleopEnabled()
        && lastActivePhase == MatchPhase.AUTONOMOUS;

    if (isPreTeleopDisabledGap) {
      if (preTeleopDisabledPauseStartUs < 0) {
        preTeleopDisabledPauseStartUs = inputs.robotTimeUs();
      }
      return;
    }

    if (preTeleopDisabledPauseStartUs >= 0) {
      preTeleopDisabledPauseUs += Math.max(0L, inputs.robotTimeUs() - preTeleopDisabledPauseStartUs);
      preTeleopDisabledPauseStartUs = -1;
    }
  }

  private double getMatchElapsedSeconds(long robotTimeUs) {
    if (matchStartUs < 0) {
      return 0.0;
    }

    long pausedUs = preTeleopDisabledPauseUs;
    if (preTeleopDisabledPauseStartUs >= 0) {
      pausedUs += Math.max(0L, robotTimeUs - preTeleopDisabledPauseStartUs);
    }

    return Math.max(0.0, (robotTimeUs - matchStartUs - pausedUs) / 1_000_000.0);
  }

  private double computePeriodTimeRemaining(MatchPhase phase, double matchElapsedS, double teleopElapsedS) {
    return switch (phase) {
      case AUTONOMOUS -> clamp(kAutoDurationS - matchElapsedS);
      case TRANSITION, SHIFT1, SHIFT2, SHIFT3, SHIFT4, ENDGAME -> computeTeleopPhaseRemaining(phase, teleopElapsedS);
      case PRE_MATCH, POST_MATCH -> 0.0;
    };
  }

  private double computeTeleopPhaseRemaining(MatchPhase phase, double teleopElapsedS) {
    return switch (phase) {
      case TRANSITION -> clamp(kTransitionDurationS - teleopElapsedS);
      case SHIFT1 -> clamp((kTransitionDurationS + kShiftDurationS) - teleopElapsedS);
      case SHIFT2 -> clamp((kTransitionDurationS + (2.0 * kShiftDurationS)) - teleopElapsedS);
      case SHIFT3 -> clamp((kTransitionDurationS + (3.0 * kShiftDurationS)) - teleopElapsedS);
      case SHIFT4 -> clamp((kTransitionDurationS + (4.0 * kShiftDurationS)) - teleopElapsedS);
      case ENDGAME -> clamp(kTeleopDurationS - teleopElapsedS);
      case PRE_MATCH, AUTONOMOUS, POST_MATCH -> 0.0;
    };
  }

  private double computeShiftTimeRemaining(MatchPhase phase, double teleopElapsedS) {
    return switch (phase) {
      case TRANSITION -> clamp(kTransitionDurationS - teleopElapsedS);
      case SHIFT1 -> clamp((kTransitionDurationS + kShiftDurationS) - teleopElapsedS);
      case SHIFT2 -> clamp((kTransitionDurationS + (2.0 * kShiftDurationS)) - teleopElapsedS);
      case SHIFT3 -> clamp((kTransitionDurationS + (3.0 * kShiftDurationS)) - teleopElapsedS);
      case SHIFT4 -> clamp((kTransitionDurationS + (4.0 * kShiftDurationS)) - teleopElapsedS);
      case PRE_MATCH, AUTONOMOUS, ENDGAME, POST_MATCH -> 0.0;
    };
  }

  private double computeShiftTimeWithBuffer(MatchPhase phase, double teleopElapsedS, boolean ourHubInactiveInShift1) {
    double shiftTimeRemainingS = computeShiftTimeRemaining(phase, teleopElapsedS);
    MatchPhase nextPhase = getNextPhase(phase);

    if (!isShiftTimerPhase(phase) || nextPhase == MatchPhase.ENDGAME || nextPhase == MatchPhase.POST_MATCH) {
      return shiftTimeRemainingS;
    }

    if (!isOurHubActive(nextPhase, ourHubInactiveInShift1)) {
      return shiftTimeRemainingS + kHubWarningDurationS;
    }

    return shiftTimeRemainingS;
  }

  private HubStatus resolveHubStatus(MatchPhase phase, double phaseElapsedS, boolean ourHubInactiveInShift1) {
    return switch (phase) {
      case PRE_MATCH, POST_MATCH -> HubStatus.HUB_NONE;
      case AUTONOMOUS, TRANSITION, ENDGAME -> HubStatus.HUB_BOTH;
      case SHIFT1, SHIFT2, SHIFT3, SHIFT4 -> {
        if (isOurHubActive(phase, ourHubInactiveInShift1)) {
          yield HubStatus.HUB_ACTIVE;
        }
        if (phaseElapsedS < kHubWarningDurationS) {
          yield HubStatus.HUB_WARNING;
        }
        yield HubStatus.HUB_INACTIVE;
      }
    };
  }

  private HeaderColor resolveHeaderColor(MatchPhase phase, HubStatus hubStatus, boolean driverOverride) {
    if (phase == MatchPhase.AUTONOMOUS || driverOverride) {
      return HeaderColor.HEADER_PURPLE;
    }
    if (hubStatus == HubStatus.HUB_WARNING) {
      return HeaderColor.HEADER_YELLOW;
    }
    if (hubStatus == HubStatus.HUB_ACTIVE || hubStatus == HubStatus.HUB_BOTH) {
      return HeaderColor.HEADER_GREEN;
    }
    return HeaderColor.HEADER_HIDDEN;
  }

  private boolean isShiftIndicatorVisible(MatchPhase phase) {
    return switch (phase) {
      case AUTONOMOUS, TRANSITION, SHIFT1, SHIFT2, SHIFT3, SHIFT4 -> true;
      case PRE_MATCH, ENDGAME, POST_MATCH -> false;
    };
  }

  private MatchPhase getNextPhase(MatchPhase phase) {
    return switch (phase) {
      case TRANSITION -> MatchPhase.SHIFT1;
      case SHIFT1 -> MatchPhase.SHIFT2;
      case SHIFT2 -> MatchPhase.SHIFT3;
      case SHIFT3 -> MatchPhase.SHIFT4;
      case SHIFT4 -> MatchPhase.ENDGAME;
      case ENDGAME, PRE_MATCH, AUTONOMOUS, POST_MATCH -> MatchPhase.POST_MATCH;
    };
  }

  private boolean isShiftTimerPhase(MatchPhase phase) {
    return switch (phase) {
      case TRANSITION, SHIFT1, SHIFT2, SHIFT3, SHIFT4 -> true;
      case PRE_MATCH, AUTONOMOUS, ENDGAME, POST_MATCH -> false;
    };
  }

  private boolean isOurHubActive(MatchPhase phase, boolean ourHubInactiveInShift1) {
    return switch (phase) {
      case SHIFT1, SHIFT3 -> !ourHubInactiveInShift1;
      case SHIFT2, SHIFT4 -> ourHubInactiveInShift1;
      case PRE_MATCH, AUTONOMOUS, TRANSITION, ENDGAME, POST_MATCH -> true;
    };
  }

  private boolean doesGameDataDeactivateUsInShift1(String gameSpecificMessage, boolean isRedAlliance) {
    return switch (gameSpecificMessage) {
      case "R" -> isRedAlliance;
      case "B" -> !isRedAlliance;
      default -> !isRedAlliance;
    };
  }

  private String resolveGameSpecificMessage(String gameSpecificMessage, boolean isRedAlliance) {
    String sanitized = gameSpecificMessage.trim().toUpperCase();
    if (!sanitized.isEmpty()) {
      return sanitized;
    }
    return isRedAlliance ? "R" : "B";
  }

  private PoseSnapshot resolvePose(Inputs inputs) {
    Pose2d pose = inputs.pose();
    if (isPoseFresh(inputs.wallClockMs(), inputs.poseLastUpdateMs()) && isPoseFinite(pose)) {
      lastValidPose = pose;
      return new PoseSnapshot(pose, true);
    }

    if (lastValidPose != null) {
      return new PoseSnapshot(lastValidPose, false);
    }

    return new PoseSnapshot(new Pose2d(), false);
  }

  private boolean isPoseFresh(long wallClockMs, long poseLastUpdateMs) {
    return poseLastUpdateMs > 0 && wallClockMs - poseLastUpdateMs <= kPoseFreshnessThresholdMs;
  }

  private boolean isPoseFinite(Pose2d pose) {
    return pose != null
        && Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }

  private double getPhaseElapsedSeconds(MatchPhase phase, double teleopElapsedS) {
    return switch (phase) {
      case TRANSITION -> teleopElapsedS;
      case SHIFT1 -> teleopElapsedS - kTransitionDurationS;
      case SHIFT2 -> teleopElapsedS - (kTransitionDurationS + kShiftDurationS);
      case SHIFT3 -> teleopElapsedS - (kTransitionDurationS + (2.0 * kShiftDurationS));
      case SHIFT4 -> teleopElapsedS - (kTransitionDurationS + (3.0 * kShiftDurationS));
      case ENDGAME -> teleopElapsedS - (kTransitionDurationS + (4.0 * kShiftDurationS));
      case PRE_MATCH, AUTONOMOUS, POST_MATCH -> 0.0;
    };
  }

  private double secondsSince(long startUs, long nowUs) {
    if (startUs < 0) {
      return 0.0;
    }
    return Math.max(0.0, (nowUs - startUs) / 1_000_000.0);
  }

  private double clamp(double value) {
    return Math.max(0.0, value);
  }

  private void resetSession() {
    sessionType = SessionType.NONE;
    matchStartUs = -1;
    teleopStartUs = -1;
    preTeleopDisabledPauseUs = 0;
    preTeleopDisabledPauseStartUs = -1;
    lastActivePhase = MatchPhase.PRE_MATCH;
    sessionArmedForReset = false;
  }

  private record PoseSnapshot(Pose2d pose, boolean valid) {
  }
}
