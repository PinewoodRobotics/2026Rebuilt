package frc.robot.constant;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.units.Units;

public class ShooterConstantsTest {
  private static final double kEpsilon = 1e-9;
  private static final double kMin = ShooterConstants.kMinCalibratedDistanceMeters;
  private static final double kMax = ShooterConstants.kMaxCalibratedDistanceMeters;

  @Test
  void FitsDoNotExtrapolateBeyondCalibratedRange() {
    assertEquals(ShooterConstants.DistanceFromTargetToTime(kMax),
        ShooterConstants.DistanceFromTargetToTime(kMax + 3.0), kEpsilon);
    assertEquals(ShooterConstants.DistanceFromTargetToVelocity(kMax).in(Units.RPM),
        ShooterConstants.DistanceFromTargetToVelocity(kMax + 3.0).in(Units.RPM), kEpsilon);
  }

  @Test
  void FitsDoNotExtrapolateBelowCalibratedRange() {
    assertEquals(ShooterConstants.DistanceFromTargetToTime(kMin),
        ShooterConstants.DistanceFromTargetToTime(0.0), kEpsilon);
    assertEquals(ShooterConstants.DistanceFromTargetToVelocity(kMin).in(Units.RPM),
        ShooterConstants.DistanceFromTargetToVelocity(0.0).in(Units.RPM), kEpsilon);
  }

  @Test
  void FitsAreUnchangedInsideCalibratedRange() {
    double distance = 3.0;
    assertEquals(0.295 * distance + 0.433, ShooterConstants.DistanceFromTargetToTime(distance), kEpsilon);
    assertEquals(135.0 * distance + 1928.0,
        ShooterConstants.DistanceFromTargetToVelocity(distance).in(Units.RPM), 1e-6);
  }

  @Test
  void IsWithinCalibratedDistanceMatchesRange() {
    assertTrue(ShooterConstants.IsWithinCalibratedDistance(kMin));
    assertTrue(ShooterConstants.IsWithinCalibratedDistance(kMax));
    assertFalse(ShooterConstants.IsWithinCalibratedDistance(kMin - 0.01));
    assertFalse(ShooterConstants.IsWithinCalibratedDistance(kMax + 0.01));
  }
}
