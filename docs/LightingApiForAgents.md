# Lighting API Guide (For AI Agents)

This document explains how to use and extend the robot lighting system safely.

## Architecture

- `LightsSubsystem` is the single owner of LED hardware writes.
- `LightsEngine` builds each frame by compositing active effects.
- `CandleLightsTransport` sends contiguous color segments to CANdle.
- `PollingCommand` is the default command on `LightsSubsystem` and runs multiple light commands each cycle.

Key files:
- `src/main/java/frc/robot/subsystem/LightsSubsystem.java`
- `src/main/java/frc/robot/util/lighting/LightsEngine.java`
- `src/main/java/frc/robot/command/util/PollingCommand.java`

## Command Model (Important)

Lighting behavior commands are intended to be **read-only signal producers**:
- They should read robot state and update effect parameters (for example progress).
- They should not directly write CANdle hardware.
- They should not own scheduling of `LightsSubsystem` directly.

Register them through:
- `LightsSubsystem.GetInstance().addLightsCommand(commandA, commandB, ...)`

Do **not** replace the lights default command in `RobotContainer`; the subsystem installs its poller internally.

## API Surface

Core types:
- `LedRange(startInclusive, endInclusive)` and `LedRange.single(index)`
- `LedColor(red, green, blue, white)` (clamped to 0..255)
- `EffectHandle<TInput>` (opaque effect id + input type marker)
- `BlendMode.OVERWRITE` and `BlendMode.ADD`
- `LightZone` enum (`ONBOARD`, `FULL_STRIP`, `EXTERNAL_STRIP`, `LEFT_HALF`, `RIGHT_HALF`)

Primary `LightsSubsystem` methods:
- `addSolid(...)`
- `addProgressBar(...)`
- `addBlink(...)`
- `addBreathe(...)`
- `addChase(...)`
- `addLarsonScanner(...)` (Knight Rider / Cylon)
- `addConvergingArrows(...)` (aiming reticle; drive exactness via `setInput(handle, 0..1)`)
- `addRainbow(...)`
- `setProgress(handle, value01)`
- `setInput(handle, typedValue)` (generic typed input channel)
- `setEnabled(handle, enabled)`
- `setPriority(handle, priority)`
- `removeEffect(handle)`
- `clearEffects()`

`LightsApi` supports typed handles:
- Effects that accept scalar progress typically return `EffectHandle<Double>`.
- Effects without runtime input typically return `EffectHandle<Void>`.
- Use `setInput(handle, value)` for generic typed updates.
- `setProgress(...)` remains available as a convenience path.

Use `LightsApi` when you only need progress bars. Use `LightsSubsystem` when you need richer effects.

## Render Semantics

- Priorities are processed ascending (`low -> high`).
- `OVERWRITE` replaces previous color for touched pixels.
- `ADD` adds channel values and clamps each channel to 255.
- If no pixels changed from prior frame, no hardware writes are sent.
- If segment count exceeds `LEDConstants.maxSolidWritesPerCycle`, adaptive compression is applied.

## How To Add A New Lighting Command

1. Create a command under `src/main/java/frc/robot/command/lighting`.
2. In `initialize()`, create effect(s) and store `EffectHandle`s.
3. In `execute()`, only update existing handles based on robot state.
4. In `end(...)`, remove effects you no longer want active (recommended).
5. Register command with `addLightsCommand(...)` in `RobotContainer`.

Minimal pattern:

```java
public class ExampleLightCommand extends Command {
  private final LightsSubsystem lights;
  private EffectHandle<Double> handle;

  public ExampleLightCommand(LightsSubsystem lights) {
    this.lights = lights;
  }

  @Override
  public void initialize() {
    handle = lights.addProgressBar(new LedRange(8, 67),
        new LedColor(0, 255, 0), new LedColor(10, 10, 10),
        10, BlendMode.OVERWRITE);
  }

  @Override
  public void execute() {
    lights.setInput(handle, 0.5);
  }

  @Override
  public void end(boolean interrupted) {
    if (handle != null) {
      lights.removeEffect(handle);
    }
  }
}
```

## Conventions For Agents

- Prefer named zones (`lights.rangeOf(LightZone.X)`) before raw indices.
- Keep LED commands deterministic and side-effect light.
- Avoid heavy allocations in `execute()`.
- Treat `EffectHandle<TInput>` as the only durable reference to a created effect.
- For overlays, use higher priority plus `BlendMode.ADD`.
- For baseline backgrounds, use lower priority plus `BlendMode.OVERWRITE`.

## Troubleshooting

- LEDs not changing: verify the command is registered via `addLightsCommand(...)`.
- Effect not visible: check priority and blend mode interactions.
- Unexpected dimming/artifacts: adaptive compression may be active (`Lights/AdaptiveCompressionActive` log key).
- Out-of-range exception: confirm index/range against `LEDConstants` (`0..399`).
