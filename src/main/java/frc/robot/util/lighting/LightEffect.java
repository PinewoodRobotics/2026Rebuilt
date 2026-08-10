package frc.robot.util.lighting;

public abstract class LightEffect<T> {
  private final LedRange range;
  private BlendMode blendMode;
  private int priority;
  private boolean enabled;

  protected LightEffect(LedRange range, int priority, BlendMode blendMode) {
    this.range = range;
    this.priority = priority;
    this.blendMode = blendMode;
    this.enabled = true;
  }

  public LedRange getRange() {
    return range;
  }

  public BlendMode getBlendMode() {
    return blendMode;
  }

  public void setBlendMode(BlendMode blendMode) {
    this.blendMode = blendMode;
  }

  public int getPriority() {
    return priority;
  }

  public void setPriority(int priority) {
    this.priority = priority;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  public boolean setProgress(double progress01) {
    return false;
  }

  public boolean updateInput(T inputArg) {
    return false;
  }

  public abstract LedColor sample(int ledIndex, double nowSeconds);
}
