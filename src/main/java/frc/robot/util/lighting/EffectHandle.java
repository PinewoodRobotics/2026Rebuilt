package frc.robot.util.lighting;

public final class EffectHandle<T> {
  private final int id;

  EffectHandle(int id) {
    this.id = id;
  }

  int id() {
    return id;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (!(obj instanceof EffectHandle<?> other)) {
      return false;
    }
    return id == other.id;
  }

  @Override
  public int hashCode() {
    return Integer.hashCode(id);
  }

  @Override
  public String toString() {
    return "EffectHandle(" + id + ")";
  }
}
