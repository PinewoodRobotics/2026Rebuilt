package frc.robot.command.shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.command.scoring.ContinuousAimCommand;
import frc.robot.constant.ShooterConstants;

public class ContinuousShooterTest {
  private static final double kEpsilon = 1e-9;

  @Test
  void CalculateShotSolutionReturnsRawAndCompensatedTargetsWhenStationary() {
    Pose2d selfPose = new Pose2d(new Translation2d(1.0, 2.0), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(4.0, 6.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds();

    ContinuousShooter.ShotSolution shotSolution = ContinuousShooter.CalculateShotSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    assertEquals(-3.0, shotSolution.targetRelative().getX(), kEpsilon);
    assertEquals(-4.0, shotSolution.targetRelative().getY(), kEpsilon);
    assertEquals(3.0, shotSolution.compensatedTargetRelative().getX(), kEpsilon);
    assertEquals(4.0, shotSolution.compensatedTargetRelative().getY(), kEpsilon);
    assertEquals(5.0, shotSolution.rawDistance(), kEpsilon);
    assertEquals(5.0, shotSolution.compensatedDistance(), kEpsilon);
  }

  @Test
  void CalculateShotSolutionAppliesLeadCompensationForRobotTranslation() {
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(3.0, 4.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(2.0, -1.0, 0.0);

    ContinuousShooter.ShotSolution shotSolution = ContinuousShooter.CalculateShotSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    double flyTime = ShooterConstants.DistanceFromTargetToTime(5.0);
    double expectedCompensatedX = 3.0 - 2.0 * flyTime;
    double expectedCompensatedY = 4.0 + flyTime;
    double expectedCompensatedDistance = Math.hypot(expectedCompensatedX, expectedCompensatedY);

    assertEquals(-3.0, shotSolution.targetRelative().getX(), kEpsilon);
    assertEquals(-4.0, shotSolution.targetRelative().getY(), kEpsilon);
    assertEquals(expectedCompensatedX, shotSolution.compensatedTargetRelative().getX(), kEpsilon);
    assertEquals(expectedCompensatedY, shotSolution.compensatedTargetRelative().getY(), kEpsilon);
    assertEquals(5.0, shotSolution.rawDistance(), kEpsilon);
    assertEquals(expectedCompensatedDistance, shotSolution.compensatedDistance(), kEpsilon);
  }

  @Test
  void CalculateShotSolutionTransformsFieldVelocityIntoRobotFrame() {
    Pose2d selfPose = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90.0));
    Translation2d targetGlobal = new Translation2d(0.0, 5.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);

    ContinuousShooter.ShotSolution shotSolution = ContinuousShooter.CalculateShotSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    double flyTime = ShooterConstants.DistanceFromTargetToTime(5.0);
    double expectedCompensatedX = 5.0;
    double expectedCompensatedY = flyTime;
    double expectedCompensatedDistance = Math.hypot(expectedCompensatedX, expectedCompensatedY);

    assertEquals(0.0, shotSolution.targetRelative().getX(), kEpsilon);
    assertEquals(-5.0, shotSolution.targetRelative().getY(), kEpsilon);
    assertEquals(expectedCompensatedX, shotSolution.compensatedTargetRelative().getX(), kEpsilon);
    assertEquals(expectedCompensatedY, shotSolution.compensatedTargetRelative().getY(), kEpsilon);
    assertEquals(5.0, shotSolution.rawDistance(), kEpsilon);
    assertEquals(expectedCompensatedDistance, shotSolution.compensatedDistance(), kEpsilon);
  }

  @Test
  void CalculateShotSolutionReturnsZeroVectorsWhenRobotAndTargetCoincideAndStationary() {
    Pose2d selfPose = new Pose2d(new Translation2d(2.0, -1.0), Rotation2d.fromDegrees(30.0));
    Translation2d targetGlobal = new Translation2d(2.0, -1.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds();

    ContinuousShooter.ShotSolution shotSolution = ContinuousShooter.CalculateShotSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    assertEquals(0.0, shotSolution.targetRelative().getX(), kEpsilon);
    assertEquals(0.0, shotSolution.targetRelative().getY(), kEpsilon);
    assertEquals(0.0, shotSolution.compensatedTargetRelative().getX(), kEpsilon);
    assertEquals(0.0, shotSolution.compensatedTargetRelative().getY(), kEpsilon);
    assertEquals(0.0, shotSolution.rawDistance(), kEpsilon);
    assertEquals(0.0, shotSolution.compensatedDistance(), kEpsilon);
  }

  @Test
  void CalculateShotSolutionUsesInterceptFlightTimeWhenTargetIsCoincidentButRobotIsMoving() {
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d();
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(1.5, -0.5, 0.0);

    ContinuousShooter.ShotSolution shotSolution = ContinuousShooter.CalculateShotSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    double flyTime = ShooterConstants.DistanceFromTargetToTime(0.0);
    double expectedCompensatedX = -1.5 * flyTime;
    double expectedCompensatedY = 0.5 * flyTime;

    assertEquals(0.0, shotSolution.targetRelative().getX(), kEpsilon);
    assertEquals(0.0, shotSolution.targetRelative().getY(), kEpsilon);
    assertEquals(expectedCompensatedX, shotSolution.compensatedTargetRelative().getX(), kEpsilon);
    assertEquals(expectedCompensatedY, shotSolution.compensatedTargetRelative().getY(), kEpsilon);
    assertEquals(0.0, shotSolution.rawDistance(), kEpsilon);
    assertEquals(Math.hypot(expectedCompensatedX, expectedCompensatedY), shotSolution.compensatedDistance(), kEpsilon);
  }

  @Test
  void CalculateShotSolutionMatchesAimMathForSameInputs() {
    Pose2d selfPose = new Pose2d(new Translation2d(1.0, -2.0), Rotation2d.fromDegrees(30.0));
    Translation2d targetGlobal = new Translation2d(4.0, 3.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(0.8, -1.2, 0.0);

    ContinuousShooter.ShotSolution shotSolution = ContinuousShooter.CalculateShotSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);
    ContinuousAimCommand.AimSolution aimSolution = ContinuousAimCommand.CalculateAimSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    assertEquals(aimSolution.distanceToTarget(), shotSolution.rawDistance(), kEpsilon);
    assertEquals(aimSolution.compensatedTargetInRobot().getX(), shotSolution.compensatedTargetRelative().getX(),
        kEpsilon);
    assertEquals(aimSolution.compensatedTargetInRobot().getY(), shotSolution.compensatedTargetRelative().getY(),
        kEpsilon);
    assertEquals(aimSolution.compensatedTargetInRobot().getNorm(), shotSolution.compensatedDistance(), kEpsilon);
  }
}
