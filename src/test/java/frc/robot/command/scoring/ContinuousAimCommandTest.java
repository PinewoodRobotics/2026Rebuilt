package frc.robot.command.scoring;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class ContinuousAimCommandTest {
  private static final double kEpsilon = 1e-9;

  @Test
  void ApplyAimTrimRotatesCommandCounterClockwiseForPositiveTrim() {
    double aimRad = Math.atan2(4.0, 3.0);

    assertEquals(aimRad, ContinuousAimCommand.ApplyAimTrim(aimRad, 0.0), kEpsilon);
    assertEquals(aimRad + Math.toRadians(2.5), ContinuousAimCommand.ApplyAimTrim(aimRad, 2.5), kEpsilon);
    assertEquals(aimRad - Math.toRadians(1.0), ContinuousAimCommand.ApplyAimTrim(aimRad, -1.0), kEpsilon);
  }
}
