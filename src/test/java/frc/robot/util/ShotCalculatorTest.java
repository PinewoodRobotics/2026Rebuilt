package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import frc.robot.constant.ShooterConstants;
import frc.robot.constant.TurretConstants;
import frc.robot.util.ShotCalculator.ShotSolution;

public class ShotCalculatorTest {
  private static final double kEpsilon = 1e-9;
  private static final Translation2d kNoOffset = new Translation2d();

  private static void assertSelfConsistent(ShotSolution solution, Rotation2d heading) {
    double compensatedDistance = solution.compensatedDistance();
    assertEquals(solution.compensatedTargetFromTurret().getNorm(), compensatedDistance, kEpsilon);
    assertEquals(ShooterConstants.DistanceFromTargetToTime(compensatedDistance), solution.flightTime(), kEpsilon);
    assertEquals(ShooterConstants.DistanceFromTargetToVelocity(compensatedDistance).in(Units.RPM),
        solution.shooterVelocity().in(Units.RPM), kEpsilon);
    assertEquals(
        Math.atan2(solution.compensatedTargetFromTurret().getY(), solution.compensatedTargetFromTurret().getX()),
        solution.turretAngle(), kEpsilon);
    assertEquals(!ShooterConstants.IsWithinCalibratedDistance(compensatedDistance), solution.distanceClamped());

    Translation2d turretVelocityRobot = solution.turretFieldVelocity().rotateBy(heading.unaryMinus());
    Translation2d expectedLead = turretVelocityRobot.times(-solution.flightTime());
    double leadTolerance = turretVelocityRobot.getNorm() * 2.0 * ShooterConstants.kLeadFlightTimeToleranceSeconds
        + kEpsilon;
    assertEquals(expectedLead.getX(), solution.leadCompensation().getX(), leadTolerance);
    assertEquals(expectedLead.getY(), solution.leadCompensation().getY(), leadTolerance);
  }

  @Test
  void StationaryRobotHasNoLead() {
    Pose2d selfPose = new Pose2d(new Translation2d(1.0, 2.0), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(2.8, 4.4);

    ShotSolution solution = ShotCalculator.Calculate(selfPose, targetGlobal, new ChassisSpeeds(), kNoOffset);

    assertEquals(1.8, solution.targetFromTurret().getX(), kEpsilon);
    assertEquals(2.4, solution.targetFromTurret().getY(), kEpsilon);
    assertEquals(3.0, solution.rawDistance(), kEpsilon);
    assertEquals(3.0, solution.compensatedDistance(), kEpsilon);
    assertEquals(0.0, solution.leadCompensation().getNorm(), kEpsilon);
    assertEquals(ShooterConstants.DistanceFromTargetToTime(3.0), solution.flightTime(), kEpsilon);
    assertEquals(Math.atan2(2.4, 1.8), solution.turretAngle(), kEpsilon);
    assertFalse(solution.distanceClamped());
    assertSelfConsistent(solution, selfPose.getRotation());
  }

  @Test
  void DefaultOverloadUsesConfiguredTurretPosition() {
    Pose2d selfPose = new Pose2d(new Translation2d(1.0, -2.0), Rotation2d.fromDegrees(30.0));
    Translation2d targetGlobal = new Translation2d(4.0, 0.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(0.8, -1.2, 0.5);

    assertEquals(
        ShotCalculator.Calculate(selfPose, targetGlobal, robotFieldSpeeds, TurretConstants.kTurretPositionInRobot),
        ShotCalculator.Calculate(selfPose, targetGlobal, robotFieldSpeeds));
  }

  @Test
  void TargetsBehindRobotAimBackwards() {
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(-2.0, -3.0);

    ShotSolution solution = ShotCalculator.Calculate(selfPose, targetGlobal, new ChassisSpeeds(), kNoOffset);

    assertEquals(Math.hypot(2.0, 3.0), solution.rawDistance(), kEpsilon);
    assertEquals(Math.atan2(-3.0, -2.0), solution.turretAngle(), kEpsilon);
  }

  @Test
  void TranslationLeadIsSelfConsistent() {
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(1.8, 2.4);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(1.0, -0.5, 0.0);

    ShotSolution solution = ShotCalculator.Calculate(selfPose, targetGlobal, robotFieldSpeeds, kNoOffset);

    assertEquals(3.0, solution.rawDistance(), kEpsilon);
    assertFalse(solution.distanceClamped());
    assertTrue(solution.compensatedDistance() > solution.rawDistance());
    assertTrue(solution.flightTime() > ShooterConstants.DistanceFromTargetToTime(3.0));
    assertSelfConsistent(solution, selfPose.getRotation());
  }

  @Test
  void FieldVelocityIsExpressedInRobotFrame() {
    Rotation2d heading = Rotation2d.fromDegrees(90.0);
    Pose2d selfPose = new Pose2d(new Translation2d(), heading);
    Translation2d targetGlobal = new Translation2d(0.0, 3.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);

    ShotSolution solution = ShotCalculator.Calculate(selfPose, targetGlobal, robotFieldSpeeds, kNoOffset);

    assertEquals(3.0, solution.targetFromTurret().getX(), kEpsilon);
    assertEquals(0.0, solution.targetFromTurret().getY(), kEpsilon);
    assertTrue(solution.leadCompensation().getY() > 0.0);
    assertSelfConsistent(solution, heading);
  }

  @Test
  void CoincidentTargetWithMovingRobotAimsAgainstMotion() {
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);

    ShotSolution solution = ShotCalculator.Calculate(selfPose, new Translation2d(), robotFieldSpeeds, kNoOffset);

    assertEquals(0.0, solution.rawDistance(), kEpsilon);
    assertEquals(Math.PI, solution.turretAngle(), kEpsilon);
    assertSelfConsistent(solution, selfPose.getRotation());
  }

  @Test
  void FastApproachStaysConsistentWhereFixedPointIterationWouldDiverge() {
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(4.0, 0.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(4.5, 0.0, 0.0);

    ShotSolution solution = ShotCalculator.Calculate(selfPose, targetGlobal, robotFieldSpeeds, kNoOffset);

    assertSelfConsistent(solution, selfPose.getRotation());
  }

  @Test
  void FastRetreatSaturatesAtCalibratedRange() {
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(3.0, 0.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(-4.5, 0.0, 0.0);

    ShotSolution solution = ShotCalculator.Calculate(selfPose, targetGlobal, robotFieldSpeeds, kNoOffset);

    assertTrue(solution.distanceClamped());
    assertEquals(ShooterConstants.DistanceFromTargetToTime(ShooterConstants.kMaxCalibratedDistanceMeters),
        solution.flightTime(), kEpsilon);
    assertSelfConsistent(solution, selfPose.getRotation());
  }

  @Test
  void DistanceBeyondCalibratedRangeIsClamped() {
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(6.0, 0.0);

    ShotSolution solution = ShotCalculator.Calculate(selfPose, targetGlobal, new ChassisSpeeds(), kNoOffset);

    assertEquals(6.0, solution.compensatedDistance(), kEpsilon);
    assertTrue(solution.distanceClamped());
    assertEquals(ShooterConstants.DistanceFromTargetToTime(ShooterConstants.kMaxCalibratedDistanceMeters),
        solution.flightTime(), kEpsilon);
    assertEquals(
        ShooterConstants.DistanceFromTargetToVelocity(ShooterConstants.kMaxCalibratedDistanceMeters).in(Units.RPM),
        solution.shooterVelocity().in(Units.RPM), kEpsilon);
  }

  @Test
  void TurretOffsetShiftsDistanceAndAngle() {
    Translation2d turretOffset = new Translation2d(0.5, 0.0);
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(2.3, 2.4);

    ShotSolution withOffset = ShotCalculator.Calculate(selfPose, targetGlobal, new ChassisSpeeds(), turretOffset);
    ShotSolution withoutOffset = ShotCalculator.Calculate(selfPose, targetGlobal, new ChassisSpeeds(), kNoOffset);

    assertEquals(1.8, withOffset.targetFromTurret().getX(), kEpsilon);
    assertEquals(2.4, withOffset.targetFromTurret().getY(), kEpsilon);
    assertEquals(3.0, withOffset.rawDistance(), kEpsilon);
    assertEquals(Math.atan2(2.4, 1.8), withOffset.turretAngle(), kEpsilon);
    assertEquals(Math.atan2(2.4, 2.3), withoutOffset.turretAngle(), kEpsilon);
  }

  @Test
  void TurretOffsetRotatesWithRobotHeading() {
    Translation2d turretOffset = new Translation2d(0.5, 0.0);
    Pose2d selfPose = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90.0));
    Translation2d targetGlobal = new Translation2d(-2.4, 2.3);

    ShotSolution solution = ShotCalculator.Calculate(selfPose, targetGlobal, new ChassisSpeeds(), turretOffset);

    assertEquals(1.8, solution.targetFromTurret().getX(), kEpsilon);
    assertEquals(2.4, solution.targetFromTurret().getY(), kEpsilon);
    assertEquals(3.0, solution.rawDistance(), kEpsilon);
    assertEquals(Math.atan2(2.4, 1.8), solution.turretAngle(), kEpsilon);
  }

  @Test
  void RotationOnlyWithoutOffsetHasNoLead() {
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(3.0, 2.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(0.0, 0.0, 3.0);

    ShotSolution solution = ShotCalculator.Calculate(selfPose, targetGlobal, robotFieldSpeeds, kNoOffset);

    assertEquals(0.0, solution.leadCompensation().getNorm(), kEpsilon);
  }

  @Test
  void RotationOnlyWithOffsetLeadsAgainstTurretVelocity() {
    Translation2d turretOffset = new Translation2d(0.5, 0.0);
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(2.5, 2.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(0.0, 0.0, 2.0);

    ShotSolution solution = ShotCalculator.Calculate(selfPose, targetGlobal, robotFieldSpeeds, turretOffset);

    assertEquals(0.0, solution.turretFieldVelocity().getX(), kEpsilon);
    assertEquals(1.0, solution.turretFieldVelocity().getY(), kEpsilon);
    assertTrue(solution.leadCompensation().getY() < -0.5);
    assertSelfConsistent(solution, selfPose.getRotation());
  }
}
