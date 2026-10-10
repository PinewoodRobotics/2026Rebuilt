package frc.robot.command.shooting;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constant.TurretConstants;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShotSolution;

public class ContinuousShooterTest {
  private static final Pose2d kOrigin = new Pose2d(new Translation2d(), new Rotation2d());
  private static final int kAimReadyMs = TurretConstants.kTurretOffByMs;

  private static ShotSolution Solve(Translation2d targetGlobal, ChassisSpeeds robotFieldSpeeds) {
    return ShotCalculator.Calculate(kOrigin, targetGlobal, robotFieldSpeeds);
  }

  @Test
  void FeedsWhenAimedSpunUpAndSolutionValid() {
    assertTrue(ContinuousShooter.CanFeed(Solve(new Translation2d(3.0, 0.0), new ChassisSpeeds()), kAimReadyMs, true));
  }

  @Test
  void DoesNotFeedWhileTurretIsStillAiming() {
    ShotSolution solution = Solve(new Translation2d(3.0, 0.0), new ChassisSpeeds());

    assertFalse(ContinuousShooter.CanFeed(solution, kAimReadyMs + 1, true));
  }

  @Test
  void DoesNotFeedBeforeShooterIsSpunUp() {
    ShotSolution solution = Solve(new Translation2d(3.0, 0.0), new ChassisSpeeds());

    assertFalse(ContinuousShooter.CanFeed(solution, kAimReadyMs, false));
  }

  @Test
  void DoesNotFeedOnNonfiniteSolution() {
    ShotSolution solution = Solve(new Translation2d(3.0, 0.0), new ChassisSpeeds(Double.NaN, 0.0, 0.0));

    assertFalse(ContinuousShooter.CanFeed(solution, kAimReadyMs, true));
  }
}
