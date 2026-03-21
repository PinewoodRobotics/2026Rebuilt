package frc.robot.subsystem;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.List;
import java.util.Locale;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedInteger;
import edu.wpi.first.networktables.TimestampedString;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constant.CommunicationConstants;
import frc.robot.constant.OrchestraConstants;

public class OrchestraSubsystem extends SubsystemBase {
  enum CommandAction {
    LOAD("load"),
    PLAY("play"),
    PAUSE("pause"),
    STOP("stop"),
    CLEAR("clear"),
    INVALID("invalid");

    private final String ntValue;

    CommandAction(String ntValue) {
      this.ntValue = ntValue;
    }

    String ntValue() {
      return ntValue;
    }
  }

  enum StatusValue {
    UNAVAILABLE("unavailable"),
    IDLE("idle"),
    LOADED("loaded"),
    PLAYING("playing"),
    PAUSED("paused"),
    ERROR("error");

    private final String ntValue;

    StatusValue(String ntValue) {
      this.ntValue = ntValue;
    }

    String ntValue() {
      return ntValue;
    }
  }

  record RequestSnapshot(long seq, String actionText, CommandAction action, String trackName) {
  }

  static final class StatusModel {
    private StatusValue status = StatusValue.IDLE;
    private String currentTrack = "";
    private String lastError = "";

    StatusValue getStatus() {
      return status;
    }

    String getCurrentTrack() {
      return currentTrack;
    }

    String getLastError() {
      return lastError;
    }

    boolean hasTrackLoaded() {
      return !currentTrack.isEmpty();
    }

    void setUnavailable(String error) {
      status = StatusValue.UNAVAILABLE;
      currentTrack = "";
      lastError = error;
    }

    void setIdle() {
      status = StatusValue.IDLE;
      currentTrack = "";
      lastError = "";
    }

    void setLoaded(String trackName) {
      status = StatusValue.LOADED;
      currentTrack = trackName;
      lastError = "";
    }

    void setPlaying() {
      if (hasTrackLoaded()) {
        status = StatusValue.PLAYING;
        lastError = "";
      }
    }

    void setPaused() {
      if (hasTrackLoaded()) {
        status = StatusValue.PAUSED;
        lastError = "";
      }
    }

    void clearTrack() {
      setIdle();
    }

    void setError(String error) {
      status = StatusValue.ERROR;
      lastError = error;
    }

    void reconcile(boolean ready, boolean isPlaying) {
      if (status == StatusValue.UNAVAILABLE) {
        return;
      }

      if (!ready) {
        if (status != StatusValue.ERROR) {
          status = StatusValue.UNAVAILABLE;
        }
        return;
      }

      if (!hasTrackLoaded() && status != StatusValue.ERROR) {
        status = StatusValue.IDLE;
        return;
      }

      if (status == StatusValue.PLAYING && !isPlaying && hasTrackLoaded()) {
        status = StatusValue.LOADED;
      }
    }
  }

  private static OrchestraSubsystem self;

  public static OrchestraSubsystem GetInstance() {
    if (self == null) {
      self = new OrchestraSubsystem();
    }

    return self;
  }

  private final Orchestra orchestra = new Orchestra();
  private final List<TalonFX> musicMotors;
  private final Path deployMusicDirectory;

  private final IntegerSubscriber requestSeqSubscriber;
  private final StringSubscriber requestActionSubscriber;
  private final StringSubscriber requestTrackNameSubscriber;

  private final BooleanPublisher readyPublisher;
  private final BooleanPublisher hasTrackLoadedPublisher;
  private final BooleanPublisher isPlayingPublisher;
  private final StringPublisher statusPublisher;
  private final StringPublisher currentTrackPublisher;
  private final DoublePublisher currentTimeSPublisher;
  private final StringPublisher lastErrorPublisher;
  private final IntegerPublisher lastCommandSeqPublisher;
  private final StringPublisher lastCommandActionPublisher;
  private final IntegerPublisher motorCountPublisher;

  private final ArrayDeque<TimestampedString> actionHistory = new ArrayDeque<>();
  private final ArrayDeque<TimestampedString> trackNameHistory = new ArrayDeque<>();
  private final StatusModel statusModel = new StatusModel();

  private TimestampedString latestAction = new TimestampedString(0, 0, "");
  private TimestampedString latestTrackName = new TimestampedString(0, 0, "");

  private long lastProcessedSeq = -1;
  private long lastCommandSeq = -1;
  private String lastCommandAction = "";
  private boolean ready;

  private OrchestraSubsystem() {
    this(NetworkTableInstance.getDefault());
  }

  OrchestraSubsystem(NetworkTableInstance tableInstance) {
    NetworkTable requestTable = Robot.getDashboard()
        .getSubTable(OrchestraConstants.kRequestTableName)
        .getSubTable(OrchestraConstants.kRequestSubtableName);
    NetworkTable statusTable = tableInstance
        .getTable(CommunicationConstants.kMatchHudRoot)
        .getSubTable(CommunicationConstants.kMatchHudStateTableName)
        .getSubTable(OrchestraConstants.kStatusTableName);

    deployMusicDirectory = Filesystem.getDeployDirectory().toPath().resolve(OrchestraConstants.kDeployMusicDirectory);

    requestSeqSubscriber = requestTable.getIntegerTopic("seq").subscribe(
        -1,
        PubSubOption.keepDuplicates(true),
        PubSubOption.pollStorage(OrchestraConstants.kCommandQueueDepth));
    requestActionSubscriber = requestTable.getStringTopic("action").subscribe(
        "",
        PubSubOption.keepDuplicates(true),
        PubSubOption.pollStorage(OrchestraConstants.kCommandQueueDepth));
    requestTrackNameSubscriber = requestTable.getStringTopic("track_name").subscribe(
        "",
        PubSubOption.keepDuplicates(true),
        PubSubOption.pollStorage(OrchestraConstants.kCommandQueueDepth));

    readyPublisher = statusTable.getBooleanTopic("ready").publish();
    hasTrackLoadedPublisher = statusTable.getBooleanTopic("has_track_loaded").publish();
    isPlayingPublisher = statusTable.getBooleanTopic("is_playing").publish();
    statusPublisher = statusTable.getStringTopic("status").publish();
    currentTrackPublisher = statusTable.getStringTopic("current_track").publish();
    currentTimeSPublisher = statusTable.getDoubleTopic("current_time_s").publish();
    lastErrorPublisher = statusTable.getStringTopic("last_error").publish();
    lastCommandSeqPublisher = statusTable.getIntegerTopic("last_command_seq").publish();
    lastCommandActionPublisher = statusTable.getStringTopic("last_command_action").publish();
    motorCountPublisher = statusTable.getIntegerTopic("motor_count").publish();

    musicMotors = SwerveSubsystem.GetInstance().getMusicTalonMotors();
    ready = initializeInstruments();
    if (ready) {
      autoplayStartupTrack();
    }
    publishState();
  }

  private boolean initializeInstruments() {
    if (musicMotors.isEmpty()) {
      String error = "No TalonFX instruments available for orchestra.";
      statusModel.setUnavailable(error);
      DriverStation.reportWarning("[Orchestra] " + error, false);
      return false;
    }

    if (!applyInstruments()) {
      return false;
    }

    statusModel.setIdle();
    return true;
  }

  private boolean applyInstruments() {
    StatusCode clearStatus = orchestra.clearInstruments();
    if (!clearStatus.isOK()) {
      fail("clearInstruments", clearStatus, false);
      return false;
    }

    for (int track = 0; track < musicMotors.size(); track++) {
      StatusCode addStatus = orchestra.addInstrument(musicMotors.get(track), track);
      if (!addStatus.isOK()) {
        fail("addInstrument(" + track + ")", addStatus, false);
        return false;
      }
    }

    return true;
  }

  static CommandAction parseAction(String rawAction) {
    if (rawAction == null) {
      return CommandAction.INVALID;
    }

    return switch (rawAction.trim().toLowerCase(Locale.ROOT)) {
      case "load" -> CommandAction.LOAD;
      case "play" -> CommandAction.PLAY;
      case "pause" -> CommandAction.PAUSE;
      case "stop" -> CommandAction.STOP;
      case "clear" -> CommandAction.CLEAR;
      default -> CommandAction.INVALID;
    };
  }

  static boolean isValidTrackName(String trackName) {
    if (trackName == null) {
      return false;
    }

    String trimmed = trackName.trim();
    if (trimmed.isEmpty()
        || !trimmed.endsWith(OrchestraConstants.kChrpExtension)
        || trimmed.contains("/")
        || trimmed.contains("\\")
        || trimmed.contains("..")) {
      return false;
    }

    Path path = Path.of(trimmed);
    return path.getNameCount() == 1 && trimmed.equals(path.getFileName().toString());
  }

  static String selectLatestValueAtOrBefore(
      ArrayDeque<TimestampedString> history,
      TimestampedString latestValue,
      long timestamp) {
    String selected = latestValue.timestamp <= timestamp ? latestValue.value : "";

    for (TimestampedString candidate : history) {
      if (candidate.timestamp <= timestamp) {
        selected = candidate.value;
      } else {
        break;
      }
    }

    return selected;
  }

  private void collectPendingRequestInputs() {
    for (TimestampedString update : requestActionSubscriber.readQueue()) {
      latestAction = update;
      actionHistory.addLast(update);
    }
    for (TimestampedString update : requestTrackNameSubscriber.readQueue()) {
      latestTrackName = update;
      trackNameHistory.addLast(update);
    }

    if (latestAction.timestamp == 0) {
      latestAction = requestActionSubscriber.getAtomic("");
    }
    if (latestTrackName.timestamp == 0) {
      latestTrackName = requestTrackNameSubscriber.getAtomic("");
    }

    trimHistory(actionHistory);
    trimHistory(trackNameHistory);
  }

  private static void trimHistory(ArrayDeque<TimestampedString> history) {
    while (history.size() > OrchestraConstants.kCommandQueueDepth * 4) {
      history.removeFirst();
    }
  }

  private RequestSnapshot snapshotRequest(TimestampedInteger seqUpdate) {
    String actionText = selectLatestValueAtOrBefore(actionHistory, latestAction, seqUpdate.timestamp)
        .trim()
        .toLowerCase(Locale.ROOT);
    String trackName = selectLatestValueAtOrBefore(trackNameHistory, latestTrackName, seqUpdate.timestamp).trim();
    return new RequestSnapshot(seqUpdate.value, actionText, parseAction(actionText), trackName);
  }

  private void processPendingRequests() {
    collectPendingRequestInputs();

    for (TimestampedInteger seqUpdate : requestSeqSubscriber.readQueue()) {
      if (seqUpdate.value <= lastProcessedSeq) {
        continue;
      }

      RequestSnapshot request = snapshotRequest(seqUpdate);
      handleRequest(request);
      lastProcessedSeq = seqUpdate.value;
    }
  }

  private void handleRequest(RequestSnapshot request) {
    lastCommandSeq = request.seq();
    lastCommandAction = request.actionText();

    switch (request.action()) {
      case LOAD:
        handleLoad(request.trackName());
        break;
      case PLAY:
        handlePlay(request.trackName());
        break;
      case PAUSE:
        handlePause();
        break;
      case STOP:
        handleStop();
        break;
      case CLEAR:
        handleClear();
        break;
      case INVALID:
        setValidationError("Unknown action: " + request.actionText());
        break;
    }
  }

  private void handleLoad(String trackName) {
    if (trackName.isEmpty()) {
      setValidationError("load requires track_name");
      return;
    }

    loadTrack(trackName);
  }

  private void handlePlay(String trackName) {
    if (!trackName.isEmpty()) {
      if (!loadTrack(trackName)) {
        return;
      }
    } else if (!statusModel.hasTrackLoaded()) {
      setValidationError("play requires track_name when no track is loaded");
      return;
    }

    if (reportIfNotOk("play", orchestra.play())) {
      statusModel.setPlaying();
    }
  }

  private void handlePause() {
    if (!statusModel.hasTrackLoaded()) {
      setValidationError("pause requires a loaded track");
      return;
    }

    if (reportIfNotOk("pause", orchestra.pause())) {
      statusModel.setPaused();
    }
  }

  private void handleStop() {
    if (!statusModel.hasTrackLoaded()) {
      statusModel.clearTrack();
      return;
    }

    if (reportIfNotOk("stop", orchestra.stop())) {
      statusModel.setLoaded(statusModel.getCurrentTrack());
    }
  }

  private void handleClear() {
    if (statusModel.hasTrackLoaded() && !reportIfNotOk("stop", orchestra.stop())) {
      return;
    }

    statusModel.clearTrack();
  }

  private boolean loadTrack(String trackName) {
    if (!ready) {
      setValidationError("orchestra is unavailable");
      return false;
    }

    if (!isValidTrackName(trackName)) {
      setValidationError("invalid track_name: " + trackName);
      return false;
    }

    Path trackPath = deployMusicDirectory.resolve(trackName).normalize();
    if (!trackPath.startsWith(deployMusicDirectory) || !Files.isRegularFile(trackPath)) {
      setValidationError("track not found: " + trackName);
      return false;
    }

    if (statusModel.hasTrackLoaded() && !reportIfNotOk("stop", orchestra.stop())) {
      return false;
    }

    String deployRelativeTrackPath = OrchestraConstants.kDeployMusicDirectory + "/" + trackName;
    StatusCode loadStatus = orchestra.loadMusic(deployRelativeTrackPath);
    if (!loadStatus.isOK()) {
      fail("loadMusic(" + deployRelativeTrackPath + ")", loadStatus, true);
      return false;
    }

    statusModel.setLoaded(trackName);
    return true;
  }

  private void autoplayStartupTrack() {
    if (!loadTrack(OrchestraConstants.kStartupTrackName)) {
      DriverStation.reportWarning(
          "[Orchestra] startup track unavailable: " + OrchestraConstants.kStartupTrackName,
          false);
      return;
    }

    if (reportIfNotOk("play startup track", orchestra.play())) {
      statusModel.setPlaying();
    }
  }

  private void setValidationError(String error) {
    if (statusModel.getStatus() == StatusValue.UNAVAILABLE) {
      statusModel.setUnavailable(error);
    } else {
      statusModel.setError(error);
    }
    DriverStation.reportWarning("[Orchestra] " + error, false);
  }

  private void fail(String action, StatusCode status, boolean preserveTrack) {
    String message = action + " failed: " + status;
    if (statusModel.getStatus() == StatusValue.UNAVAILABLE && !preserveTrack) {
      statusModel.setUnavailable(message);
    } else {
      statusModel.setError(message);
    }
    DriverStation.reportWarning("[Orchestra] " + message, false);
  }

  private boolean reportIfNotOk(String action, StatusCode status) {
    if (!status.isOK()) {
      fail(action, status, true);
      return false;
    }
    return true;
  }

  private boolean isPlaying() {
    return statusModel.hasTrackLoaded() && orchestra.isPlaying();
  }

  private double getCurrentTimeS() {
    return statusModel.hasTrackLoaded() ? orchestra.getCurrentTime() : 0.0;
  }

  private void publishState() {
    boolean hasTrackLoaded = statusModel.hasTrackLoaded();
    boolean isPlaying = isPlaying();

    readyPublisher.set(ready);
    hasTrackLoadedPublisher.set(hasTrackLoaded);
    isPlayingPublisher.set(isPlaying);
    statusPublisher.set(statusModel.getStatus().ntValue());
    currentTrackPublisher.set(statusModel.getCurrentTrack());
    currentTimeSPublisher.set(getCurrentTimeS());
    lastErrorPublisher.set(statusModel.getLastError());
    lastCommandSeqPublisher.set(lastCommandSeq);
    lastCommandActionPublisher.set(lastCommandAction);
    motorCountPublisher.set(musicMotors.size());
  }

  @Override
  public void periodic() {
    processPendingRequests();
    statusModel.reconcile(ready, isPlaying());

    Logger.recordOutput("Orchestra/Ready", ready);
    Logger.recordOutput("Orchestra/HasTrackLoaded", statusModel.hasTrackLoaded());
    Logger.recordOutput("Orchestra/IsPlaying", isPlaying());
    Logger.recordOutput("Orchestra/Status", statusModel.getStatus().ntValue());
    Logger.recordOutput("Orchestra/CurrentTrack", statusModel.getCurrentTrack());
    Logger.recordOutput("Orchestra/CurrentTimeS", getCurrentTimeS());
    Logger.recordOutput("Orchestra/LastError", statusModel.getLastError());
    Logger.recordOutput("Orchestra/LastCommandSeq", lastCommandSeq);
    Logger.recordOutput("Orchestra/LastCommandAction", lastCommandAction);
    Logger.recordOutput("Orchestra/MotorCount", musicMotors.size());

    publishState();
  }
}
