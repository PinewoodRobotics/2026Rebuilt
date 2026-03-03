package frc.robot.util.lighting;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class LightsEngine {
  public record LedSegment(int startInclusive, int endInclusive, LedColor color) {
  }

  public record RenderResult(
      List<LedSegment> segments,
      boolean adaptiveCompressionActive,
      int changedLedCount,
      int activeEffectCount) {
    public boolean hasWrites() {
      return !segments.isEmpty();
    }
  }

  private final int ledCount;
  private final int maxSolidWritesPerCycle;

  private final int[] currentRed;
  private final int[] currentGreen;
  private final int[] currentBlue;
  private final int[] currentWhite;

  private final int[] previousRed;
  private final int[] previousGreen;
  private final int[] previousBlue;
  private final int[] previousWhite;

  private final int[] layerRed;
  private final int[] layerGreen;
  private final int[] layerBlue;
  private final int[] layerWhite;
  private final boolean[] layerTouched;

  private final Map<Integer, LightEffect<?>> effectsById = new HashMap<>();
  private int nextEffectId = 1;

  public LightsEngine(int ledCount, int maxSolidWritesPerCycle) {
    if (ledCount <= 0) {
      throw new IllegalArgumentException("ledCount must be > 0");
    }

    this.ledCount = ledCount;
    this.maxSolidWritesPerCycle = Math.max(1, maxSolidWritesPerCycle);

    currentRed = new int[ledCount];
    currentGreen = new int[ledCount];
    currentBlue = new int[ledCount];
    currentWhite = new int[ledCount];

    previousRed = new int[ledCount];
    previousGreen = new int[ledCount];
    previousBlue = new int[ledCount];
    previousWhite = new int[ledCount];

    layerRed = new int[ledCount];
    layerGreen = new int[ledCount];
    layerBlue = new int[ledCount];
    layerWhite = new int[ledCount];
    layerTouched = new boolean[ledCount];
  }

  public <T> EffectHandle<T> addEffect(LightEffect<T> effect) {
    int id = nextEffectId++;
    effectsById.put(id, effect);
    return new EffectHandle<>(id);
  }

  public boolean removeEffect(EffectHandle<?> handle) {
    return effectsById.remove(handle.id()) != null;
  }

  public void clearEffects() {
    effectsById.clear();
  }

  public boolean setEnabled(EffectHandle<?> handle, boolean enabled) {
    LightEffect<?> effect = effectsById.get(handle.id());
    if (effect == null) {
      return false;
    }
    effect.setEnabled(enabled);
    return true;
  }

  public boolean setPriority(EffectHandle<?> handle, int priority) {
    LightEffect<?> effect = effectsById.get(handle.id());
    if (effect == null) {
      return false;
    }
    effect.setPriority(priority);
    return true;
  }

  public boolean setProgress(EffectHandle<?> handle, double progress01) {
    LightEffect<?> effect = effectsById.get(handle.id());
    if (effect == null) {
      return false;
    }
    return effect.setProgress(progress01);
  }

  public <T> boolean setInput(EffectHandle<T> handle, T inputArg) {
    LightEffect<?> effect = effectsById.get(handle.id());
    if (effect == null) {
      return false;
    }
    return castAndUpdate(effect, inputArg);
  }

  @SuppressWarnings("unchecked")
  private static <T> boolean castAndUpdate(LightEffect<?> effect, T inputArg) {
    try {
      return ((LightEffect<T>) effect).updateInput(inputArg);
    } catch (ClassCastException ex) {
      return false;
    }
  }

  public RenderResult render(double nowSeconds) {
    clearCurrentFrame();

    List<Map.Entry<Integer, LightEffect<?>>> activeEffects = new ArrayList<>();
    for (Map.Entry<Integer, LightEffect<?>> entry : effectsById.entrySet()) {
      if (entry.getValue().isEnabled()) {
        activeEffects.add(entry);
      }
    }

    activeEffects.sort(
        Comparator
            .comparingInt((Map.Entry<Integer, LightEffect<?>> entry) -> entry.getValue().getPriority())
            .thenComparingInt(Map.Entry::getKey));

    for (Map.Entry<Integer, LightEffect<?>> entry : activeEffects) {
      applyEffect(entry.getValue(), nowSeconds);
    }

    DiffResult diff = computeDiffSegments();
    boolean adaptiveCompressionActive = false;
    List<LedSegment> segmentsToWrite = diff.segments;

    if (segmentsToWrite.size() > maxSolidWritesPerCycle) {
      adaptiveCompressionActive = true;
      segmentsToWrite = buildCompressedSegments(maxSolidWritesPerCycle);
    }

    copyCurrentToPrevious();

    return new RenderResult(
        List.copyOf(segmentsToWrite),
        adaptiveCompressionActive,
        diff.changedLedCount,
        activeEffects.size());
  }

  public int getActiveEffectCount() {
    int count = 0;
    for (LightEffect<?> effect : effectsById.values()) {
      if (effect.isEnabled()) {
        count++;
      }
    }
    return count;
  }

  private void applyEffect(LightEffect<?> effect, double nowSeconds) {
    LedRange range = effect.getRange();
    int start = range.startInclusive();
    int end = range.endInclusive();

    for (int i = start; i <= end; i++) {
      layerTouched[i] = false;
    }

    for (int i = start; i <= end; i++) {
      LedColor sample = effect.sample(i, nowSeconds);
      if (sample == null) {
        continue;
      }

      layerTouched[i] = true;
      layerRed[i] = sample.red();
      layerGreen[i] = sample.green();
      layerBlue[i] = sample.blue();
      layerWhite[i] = sample.white();
    }

    if (effect.getBlendMode() == BlendMode.OVERWRITE) {
      for (int i = start; i <= end; i++) {
        if (!layerTouched[i]) {
          continue;
        }
        currentRed[i] = layerRed[i];
        currentGreen[i] = layerGreen[i];
        currentBlue[i] = layerBlue[i];
        currentWhite[i] = layerWhite[i];
      }
      return;
    }

    for (int i = start; i <= end; i++) {
      if (!layerTouched[i]) {
        continue;
      }
      currentRed[i] = LedColor.clamp(currentRed[i] + layerRed[i]);
      currentGreen[i] = LedColor.clamp(currentGreen[i] + layerGreen[i]);
      currentBlue[i] = LedColor.clamp(currentBlue[i] + layerBlue[i]);
      currentWhite[i] = LedColor.clamp(currentWhite[i] + layerWhite[i]);
    }
  }

  private DiffResult computeDiffSegments() {
    List<LedSegment> segments = new ArrayList<>();
    int changedLedCount = 0;

    int i = 0;
    while (i < ledCount) {
      if (!isChanged(i)) {
        i++;
        continue;
      }

      int start = i;
      int red = currentRed[i];
      int green = currentGreen[i];
      int blue = currentBlue[i];
      int white = currentWhite[i];

      changedLedCount++;
      i++;

      while (i < ledCount
          && isChanged(i)
          && currentRed[i] == red
          && currentGreen[i] == green
          && currentBlue[i] == blue
          && currentWhite[i] == white) {
        changedLedCount++;
        i++;
      }

      segments.add(new LedSegment(start, i - 1, new LedColor(red, green, blue, white)));
    }

    return new DiffResult(segments, changedLedCount);
  }

  private List<LedSegment> buildCompressedSegments(int maxSegmentCount) {
    int segmentCount = Math.max(1, Math.min(maxSegmentCount, ledCount));
    List<LedSegment> compressed = new ArrayList<>(segmentCount);

    for (int bucket = 0; bucket < segmentCount; bucket++) {
      int start = bucket * ledCount / segmentCount;
      int end = ((bucket + 1) * ledCount / segmentCount) - 1;
      if (end < start) {
        continue;
      }

      long red = 0;
      long green = 0;
      long blue = 0;
      long white = 0;

      int length = end - start + 1;
      for (int i = start; i <= end; i++) {
        red += currentRed[i];
        green += currentGreen[i];
        blue += currentBlue[i];
        white += currentWhite[i];
      }

      LedColor average = new LedColor(
          (int) Math.round(red / (double) length),
          (int) Math.round(green / (double) length),
          (int) Math.round(blue / (double) length),
          (int) Math.round(white / (double) length));

      compressed.add(new LedSegment(start, end, average));
    }

    return compressed;
  }

  private boolean isChanged(int ledIndex) {
    return currentRed[ledIndex] != previousRed[ledIndex]
        || currentGreen[ledIndex] != previousGreen[ledIndex]
        || currentBlue[ledIndex] != previousBlue[ledIndex]
        || currentWhite[ledIndex] != previousWhite[ledIndex];
  }

  private void clearCurrentFrame() {
    for (int i = 0; i < ledCount; i++) {
      currentRed[i] = 0;
      currentGreen[i] = 0;
      currentBlue[i] = 0;
      currentWhite[i] = 0;
    }
  }

  private void copyCurrentToPrevious() {
    for (int i = 0; i < ledCount; i++) {
      previousRed[i] = currentRed[i];
      previousGreen[i] = currentGreen[i];
      previousBlue[i] = currentBlue[i];
      previousWhite[i] = currentWhite[i];
    }
  }

  private record DiffResult(List<LedSegment> segments, int changedLedCount) {
  }
}
