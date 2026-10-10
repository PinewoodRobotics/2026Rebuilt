package frc.robot.subsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.units.Units;
import frc.robot.constant.ShooterConstants;

public class ShooterSubsystemTest {
  private static final double kEpsilon = 1e-9;
  private static final double kMinRpm = ShooterConstants.kShooterMinVelocity.in(Units.RPM);
  private static final double kMaxRpm = ShooterConstants.kShooterMaxVelocity.in(Units.RPM);

  @Test
  void ClampCommandedRpmEnforcesMaxCap() {
    assertEquals(kMaxRpm, ShooterSubsystem.ClampCommandedRpm(kMaxRpm + 500.0), kEpsilon);
  }

  @Test
  void ClampCommandedRpmRaisesNonzeroRequestsToMin() {
    assertEquals(kMinRpm, ShooterSubsystem.ClampCommandedRpm(kMinRpm / 2.0), kEpsilon);
  }

  @Test
  void ClampCommandedRpmPassesThroughInRangeRequests() {
    double inRange = (kMinRpm + kMaxRpm) / 2.0;
    assertEquals(inRange, ShooterSubsystem.ClampCommandedRpm(inRange), kEpsilon);
  }

  @Test
  void ClampCommandedRpmKeepsZeroAsStop() {
    assertEquals(0.0, ShooterSubsystem.ClampCommandedRpm(0.0), kEpsilon);
  }

  @Test
  void ClampCommandedRpmPreservesDirection() {
    assertEquals(-kMaxRpm, ShooterSubsystem.ClampCommandedRpm(-(kMaxRpm + 500.0)), kEpsilon);
  }
}
