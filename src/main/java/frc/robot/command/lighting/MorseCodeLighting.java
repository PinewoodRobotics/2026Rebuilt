package frc.robot.command.lighting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.LEDConstants;
import frc.robot.subsystem.LightsSubsystem;
import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.EffectHandle;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightsApi;

public class MorseCodeLighting extends Command {
  private static final String kMessage = "Jay, STOP SAYING THE N WORD!";
  private static final LedRange kRange = new LedRange(LEDConstants.onboardStartIndex, LEDConstants.ledEndIndex);
  private static final LedColor kOnColor = new LedColor(0, 200, 255, 0);
  private static final LedColor kOffColor = LedColor.BLACK;
  private static final double kMorseUnitSeconds = 0.12;
  private static final int kPriority = 50;

  private final LightsApi lightsApi;
  private EffectHandle<String> morseHandle;

  public MorseCodeLighting() {
    this(LightsSubsystem.GetInstance());
  }

  public MorseCodeLighting(LightsSubsystem lightsSubsystem) {
    super();
    this.lightsApi = lightsSubsystem;
  }

  @Override
  public void initialize() {
    morseHandle = lightsApi.addMorseCode(
        kRange,
        kMessage,
        kOnColor,
        kOffColor,
        kMorseUnitSeconds,
        kPriority,
        BlendMode.ADD);

    if (morseHandle != null) {
      lightsApi.setInput(morseHandle, kMessage);
    }
  }

  @Override
  public void execute() {
    // Morse rendering is handled internally by the effect.
  }

  @Override
  public void end(boolean interrupted) {
    if (morseHandle != null) {
      lightsApi.removeEffect(morseHandle);
      morseHandle = null;
    }
  }
}
