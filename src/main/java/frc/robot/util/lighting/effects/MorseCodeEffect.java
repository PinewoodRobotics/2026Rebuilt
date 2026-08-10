package frc.robot.util.lighting.effects;

import frc.robot.util.lighting.BlendMode;
import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LedRange;
import frc.robot.util.lighting.LightEffect;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class MorseCodeEffect extends LightEffect<String> {
  private static final Map<Character, String> MORSE_TABLE = Map.ofEntries(
      Map.entry('A', ".-"),
      Map.entry('B', "-..."),
      Map.entry('C', "-.-."),
      Map.entry('D', "-.."),
      Map.entry('E', "."),
      Map.entry('F', "..-."),
      Map.entry('G', "--."),
      Map.entry('H', "...."),
      Map.entry('I', ".."),
      Map.entry('J', ".---"),
      Map.entry('K', "-.-"),
      Map.entry('L', ".-.."),
      Map.entry('M', "--"),
      Map.entry('N', "-."),
      Map.entry('O', "---"),
      Map.entry('P', ".--."),
      Map.entry('Q', "--.-"),
      Map.entry('R', ".-."),
      Map.entry('S', "..."),
      Map.entry('T', "-"),
      Map.entry('U', "..-"),
      Map.entry('V', "...-"),
      Map.entry('W', ".--"),
      Map.entry('X', "-..-"),
      Map.entry('Y', "-.--"),
      Map.entry('Z', "--.."),
      Map.entry('0', "-----"),
      Map.entry('1', ".----"),
      Map.entry('2', "..---"),
      Map.entry('3', "...--"),
      Map.entry('4', "....-"),
      Map.entry('5', "....."),
      Map.entry('6', "-...."),
      Map.entry('7', "--..."),
      Map.entry('8', "---.."),
      Map.entry('9', "----."));

  private final LedColor onColor;
  private final LedColor offColor;
  private final double unitSeconds;
  private List<Segment> segments;
  private double cycleDurationSeconds;

  private record Segment(boolean on, double durationSeconds) {
  }

  public MorseCodeEffect(
      LedRange range,
      String message,
      LedColor onColor,
      LedColor offColor,
      double unitSeconds,
      int priority,
      BlendMode blendMode) {
    super(range, priority, blendMode);
    this.onColor = onColor;
    this.offColor = offColor;
    this.unitSeconds = unitSeconds > 0.0 ? unitSeconds : 0.1;
    rebuildSequence(message);
  }

  @Override
  public boolean updateInput(String message) {
    rebuildSequence(message);
    return true;
  }

  @Override
  public LedColor sample(int ledIndex, double nowSeconds) {
    if (segments.isEmpty()) {
      return offColor;
    }

    double cycleTime = nowSeconds % cycleDurationSeconds;
    for (Segment segment : segments) {
      if (cycleTime < segment.durationSeconds()) {
        return segment.on() ? onColor : offColor;
      }
      cycleTime -= segment.durationSeconds();
    }

    return offColor;
  }

  private void rebuildSequence(String message) {
    this.segments = buildSegments(message == null ? "" : message);
    this.cycleDurationSeconds = 0.0;
    for (Segment segment : segments) {
      this.cycleDurationSeconds += segment.durationSeconds();
    }
  }

  private List<Segment> buildSegments(String message) {
    List<Segment> built = new ArrayList<>();
    String[] words = message.toUpperCase().trim().split("\\s+");

    for (int wordIdx = 0; wordIdx < words.length; wordIdx++) {
      String word = words[wordIdx];
      if (word.isEmpty()) {
        continue;
      }

      for (int charIdx = 0; charIdx < word.length(); charIdx++) {
        char currentChar = word.charAt(charIdx);
        String morse = MORSE_TABLE.get(currentChar);
        if (morse == null) {
          continue;
        }

        for (int symbolIdx = 0; symbolIdx < morse.length(); symbolIdx++) {
          char symbol = morse.charAt(symbolIdx);
          double onDurationUnits = symbol == '-' ? 3.0 : 1.0;
          built.add(new Segment(true, onDurationUnits * unitSeconds));

          if (symbolIdx < morse.length() - 1) {
            built.add(new Segment(false, 1.0 * unitSeconds));
          }
        }

        if (charIdx < word.length() - 1) {
          built.add(new Segment(false, 3.0 * unitSeconds));
        }
      }

      if (wordIdx < words.length - 1) {
        built.add(new Segment(false, 7.0 * unitSeconds));
      }
    }

    if (built.isEmpty()) {
      built.add(new Segment(false, unitSeconds));
    }

    return built;
  }
}
