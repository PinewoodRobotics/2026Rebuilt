package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.command.util.PollingCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.LEDConstants;
import frc.robot.util.lighting.effects.BlinkEffect;
import frc.robot.util.lighting.effects.BreatheEffect;
import frc.robot.util.lighting.effects.ChaseEffect;
import frc.robot.util.lighting.effects.ConvergingArrowsEffect;
import frc.robot.util.lighting.effects.LarsonScannerEffect;
import frc.robot.util.lighting.effects.MorseCodeEffect;
import frc.robot.util.lighting.effects.ProgressBarEffect;
import frc.robot.util.lighting.effects.RainbowEffect;
import frc.robot.util.lighting.effects.SolidEffect;
import frc.robot.util.lighting.internal.CandleLightsTransport;
import frc.robot.util.lighting.internal.LightsTransport;
import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.EffectHandle;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightZone;
import frc.robot.util.lighting.LightsApi;
import frc.robot.util.lighting.LightsEngine;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public final class LightsSubsystem extends SubsystemBase implements LightsApi {
  private static LightsSubsystem instance;

  public static LightsSubsystem GetInstance() {
    if (instance == null) {
      instance = new LightsSubsystem();
    }
    return instance;
  }

  private final CANdle candle;
  private final LightsEngine engine;
  private final LightsTransport transport;
  private final List<Command> pollingCommands;
  private final PollingCommand pollingCommand;

  public LightsSubsystem() {

    this(
        new CANdle(LEDConstants.candleCANId),
        new LightsEngine(LEDConstants.ledCount, LEDConstants.maxSolidWritesPerCycle));
  }

  LightsSubsystem(CANdle candle, LightsEngine engine) {
    this(candle, engine, new CandleLightsTransport(candle));
  }

  LightsSubsystem(CANdle candle, LightsEngine engine, LightsTransport transport) {
    this.candle = candle;
    this.engine = engine;
    this.transport = transport;
    this.pollingCommands = new ArrayList<>();

    this.pollingCommand = new PollingCommand(this, () -> pollingCommands);
    super.setDefaultCommand(pollingCommand);
  }

  public LedRange rangeOf(LightZone zone) {
    return zone.range();
  }

  @Override
  public EffectHandle<Void> addSolid(LedRange range, LedColor color, int priority, BlendMode blend) {
    return engine.addEffect(new SolidEffect(range, color, priority, blend));
  }

  @Override
  public EffectHandle<Double> addProgressBar(
      LedRange range,
      LedColor fill,
      LedColor emptyOrNull,
      int priority,
      BlendMode blend) {
    return engine.addEffect(new ProgressBarEffect(range, fill, emptyOrNull, priority, blend));
  }

  @Override
  public EffectHandle<Void> addBlink(
      LedRange range,
      LedColor on,
      LedColor off,
      double hz,
      int priority,
      BlendMode blend) {
    return engine.addEffect(new BlinkEffect(range, on, off, hz, priority, blend));
  }

  @Override
  public EffectHandle<Void> addBreathe(
      LedRange range,
      LedColor color,
      double hz,
      double minScalar,
      double maxScalar,
      int priority,
      BlendMode blend) {
    return engine.addEffect(new BreatheEffect(range, color, hz, minScalar, maxScalar, priority, blend));
  }

  @Override
  public EffectHandle<Void> addChase(
      LedRange range,
      LedColor color,
      int width,
      double hz,
      boolean wrap,
      int priority,
      BlendMode blend) {
    return engine.addEffect(new ChaseEffect(range, color, width, hz, wrap, priority, blend));
  }

  @Override
  public EffectHandle<Void> addRainbow(
      LedRange range,
      double hz,
      double brightness,
      int priority,
      BlendMode blend) {
    return engine.addEffect(new RainbowEffect(range, hz, brightness, priority, blend));
  }

  @Override
  public EffectHandle<Void> addLarsonScanner(
      LedRange range,
      LedColor color,
      int eyeWidth,
      int trailLength,
      double hz,
      int priority,
      BlendMode blend) {
    return engine.addEffect(new LarsonScannerEffect(range, color, eyeWidth, trailLength, hz, priority, blend));
  }

  @Override
  public EffectHandle<String> addMorseCode(
      LedRange range,
      String message,
      LedColor onColor,
      LedColor offColor,
      double unitSeconds,
      int priority,
      BlendMode blend) {
    return engine.addEffect(
        new MorseCodeEffect(range, message, onColor, offColor, unitSeconds, priority, blend));
  }

  @Override
  public EffectHandle<Double> addConvergingArrows(
      LedRange range,
      LedColor color,
      boolean wedge,
      int priority,
      BlendMode blend) {
    return engine.addEffect(new ConvergingArrowsEffect(range, color, wedge, priority, blend));
  }

  @Override
  public <T> boolean setInput(EffectHandle<T> handle, T inputArg) {
    return engine.setInput(handle, inputArg);
  }

  @Override
  public boolean setProgress(EffectHandle<?> handle, double progress01) {
    return engine.setProgress(handle, progress01);
  }

  @Override
  public boolean setEnabled(EffectHandle<?> handle, boolean enabled) {
    return engine.setEnabled(handle, enabled);
  }

  @Override
  public boolean setPriority(EffectHandle<?> handle, int priority) {
    return engine.setPriority(handle, priority);
  }

  @Override
  public boolean removeEffect(EffectHandle<?> handle) {
    return engine.removeEffect(handle);
  }

  @Override
  public void clearEffects() {
    engine.clearEffects();
  }

  /**
   * Adds commands to the polling list run by the default {@link PollingCommand}.
   * Does not replace the default command; the single default is always the
   * poller that runs these.
   */
  public void addLightsCommand(Command... commands) {
    for (Command command : commands) {
      if (pollingCommand.isCommandAlreadyInserted(command)) {
        continue;
      }

      pollingCommands.add(command);
    }
  }

  /**
   * Adds commands to the polling list run by the default {@link PollingCommand}.
   * Does not replace the default command; the single default is always the
   * poller that runs these.
   */
  public void addLightsCommands(Command command) {
    addLightsCommand(command);
  }

  @Override
  public void periodic() {
    LightsEngine.RenderResult result = engine.render(Timer.getFPGATimestamp());

    if (result.hasWrites()) {
      transport.writeSegments(result.segments());
    }

    Logger.recordOutput("Lights/ChangedLedCount", result.changedLedCount());
    Logger.recordOutput("Lights/SegmentWriteCount", result.segments().size());
    Logger.recordOutput("Lights/AdaptiveCompressionActive", result.adaptiveCompressionActive());
    Logger.recordOutput("Lights/ActiveEffectCount", result.activeEffectCount());
  }

  public CANdle getCandle() {
    return candle;
  }

}
