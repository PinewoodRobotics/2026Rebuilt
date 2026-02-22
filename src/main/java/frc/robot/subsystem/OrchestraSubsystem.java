package frc.robot.subsystem;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.BotConstants;
import frc.robot.constant.BotConstants.RobotVariant;
import frc.robot.constant.OrchestraConstants;

public class OrchestraSubsystem extends SubsystemBase {
  private static OrchestraSubsystem self;

  private final Orchestra orchestra = new Orchestra();
  private final List<TalonFX> musicMotors;
  private int currentTrackIndex = 0;
  private boolean loaded = false;

  public static OrchestraSubsystem GetInstance() {
    if (self == null) {
      self = new OrchestraSubsystem();
    }

    return self;
  }

  private OrchestraSubsystem() {
    if (BotConstants.robotType != RobotVariant.ABOT) {
      DriverStation.reportWarning("[Orchestra] Skipping init: robot variant is not ABOT.", false);
      musicMotors = List.of();
      return;
    }

    musicMotors = SwerveSubsystem.GetInstance().getMusicTalonMotors();
    if (musicMotors.isEmpty()) {
      DriverStation.reportWarning(
          "[Orchestra] Skipping init: no TalonFX motors available for Orchestra.",
          false);
      return;
    }

    if (OrchestraConstants.kPlaylist.isEmpty()) {
      DriverStation.reportWarning("[Orchestra] Skipping init: playlist is empty.", false);
      return;
    }

    currentTrackIndex = normalizeTrackIndex(OrchestraConstants.kDefaultTrackIndex);
    loadAndPlayCurrent();
  }

  public void playMario() {
    playTrackIfPresent("music/mario-overworld.chrp");
  }

  public void playKpop() {
    playTrackIfPresent("music/kpop-golden.chrp");
  }

  public void nextTrackAndPlay() {
    if (OrchestraConstants.kPlaylist.isEmpty()) {
      return;
    }

    currentTrackIndex = normalizeTrackIndex(currentTrackIndex + 1);
    loadAndPlayCurrent();
  }

  public void previousTrackAndPlay() {
    if (OrchestraConstants.kPlaylist.isEmpty()) {
      return;
    }

    currentTrackIndex = normalizeTrackIndex(currentTrackIndex - 1);
    loadAndPlayCurrent();
  }

  public int getCurrentTrackIndex() {
    return currentTrackIndex;
  }

  public int getTrackCount() {
    return OrchestraConstants.kPlaylist.size();
  }

  public void loadAndPlayCurrent() {
    if (OrchestraConstants.kPlaylist.isEmpty()) {
      return;
    }
    loadAndPlay(OrchestraConstants.kPlaylist.get(currentTrackIndex));
  }

  public void loadAndPlay(String musicPath) {
    if (musicMotors.isEmpty()) {
      return;
    }

    reportIfNotOk("clearInstruments", orchestra.clearInstruments());
    for (int track = 0; track < musicMotors.size(); track++) {
      reportIfNotOk("addInstrument(track=" + track + ")", orchestra.addInstrument(musicMotors.get(track), track));
    }

    StatusCode loadStatus = orchestra.loadMusic(musicPath);
    loaded = loadStatus.isOK();
    reportIfNotOk("loadMusic(" + musicPath + ")", loadStatus);
    if (loaded) {
      playFromStart();
    }
  }

  public void togglePlayFromStart() {
    if (!loaded) {
      return;
    }

    if (orchestra.isPlaying()) {
      reportIfNotOk("stop", orchestra.stop());
    } else {
      playFromStart();
    }
  }

  public void playFromStart() {
    if (!loaded) {
      return;
    }

    reportIfNotOk("stop", orchestra.stop());
    reportIfNotOk("play", orchestra.play());
  }

  public void pause() {
    if (!loaded) {
      return;
    }

    reportIfNotOk("pause", orchestra.pause());
  }

  public void stop() {
    if (!loaded) {
      return;
    }

    reportIfNotOk("stop", orchestra.stop());
  }

  public boolean isPlaying() {
    return loaded && orchestra.isPlaying();
  }

  private void playTrackIfPresent(String trackPath) {
    int index = OrchestraConstants.kPlaylist.indexOf(trackPath);
    if (index < 0) {
      DriverStation.reportWarning("[Orchestra] Track not found in playlist: " + trackPath, false);
      return;
    }

    currentTrackIndex = index;
    loadAndPlayCurrent();
  }

  private static int normalizeTrackIndex(int index) {
    int size = OrchestraConstants.kPlaylist.size();
    if (size == 0) {
      return 0;
    }

    int wrapped = index % size;
    if (wrapped < 0) {
      wrapped += size;
    }
    return wrapped;
  }

  private static void reportIfNotOk(String action, StatusCode status) {
    if (!status.isOK()) {
      DriverStation.reportWarning("[Orchestra] " + action + " failed: " + status, false);
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Orchestra/Loaded", loaded);
    Logger.recordOutput("Orchestra/Playing", isPlaying());
    Logger.recordOutput("Orchestra/MotorCount", musicMotors.size());
    Logger.recordOutput("Orchestra/TrackIndex", currentTrackIndex);
    Logger.recordOutput("Orchestra/TrackCount", OrchestraConstants.kPlaylist.size());
    Logger.recordOutput("Orchestra/CurrentTimeSec", orchestra.getCurrentTime());
  }
}
