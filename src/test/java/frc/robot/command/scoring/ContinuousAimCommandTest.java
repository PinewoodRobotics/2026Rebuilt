package frc.robot.command.scoring;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.command.shooting.ContinuousShooter;
import frc.robot.constant.ShooterConstants;

public class ContinuousAimCommandTest {
  private static final double kEpsilon = 1e-9;

  @Test
  void CalculateAimSolutionReturnsExpectedValuesWhenStationary() {
    Pose2d selfPose = new Pose2d(new Translation2d(1.0, 2.0), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(4.0, 6.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds();

    ContinuousAimCommand.AimSolution aimSolution = ContinuousAimCommand.CalculateAimSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    assertEquals(3.0, aimSolution.targetInRobotFrame().getX(), kEpsilon);
    assertEquals(4.0, aimSolution.targetInRobotFrame().getY(), kEpsilon);
    assertEquals(5.0, aimSolution.distanceToTarget(), kEpsilon);
    assertEquals(ShooterConstants.DistanceFromTargetToTime(5.0), aimSolution.flyTime(), kEpsilon);
    assertEquals(3.0, aimSolution.compensatedTargetInRobot().getX(), kEpsilon);
    assertEquals(4.0, aimSolution.compensatedTargetInRobot().getY(), kEpsilon);
    assertEquals(0.0, aimSolution.leadCompensation().getX(), kEpsilon);
    assertEquals(0.0, aimSolution.leadCompensation().getY(), kEpsilon);
    assertEquals(Math.atan2(4.0, 3.0), aimSolution.turretAngle(), kEpsilon);
  }

  @Test
  void CalculateAimSolutionAppliesLeadCompensationForRobotTranslation() {
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(3.0, 4.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(2.0, -1.0, 0.0);

    ContinuousAimCommand.AimSolution aimSolution = ContinuousAimCommand.CalculateAimSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    double flyTime = ShooterConstants.DistanceFromTargetToTime(5.0);
    double expectedCompensatedX = 3.0 - 2.0 * flyTime;
    double expectedCompensatedY = 4.0 + flyTime;

    assertEquals(3.0, aimSolution.targetInRobotFrame().getX(), kEpsilon);
    assertEquals(4.0, aimSolution.targetInRobotFrame().getY(), kEpsilon);
    assertEquals(5.0, aimSolution.distanceToTarget(), kEpsilon);
    assertEquals(flyTime, aimSolution.flyTime(), kEpsilon);
    assertEquals(expectedCompensatedX, aimSolution.compensatedTargetInRobot().getX(), kEpsilon);
    assertEquals(expectedCompensatedY, aimSolution.compensatedTargetInRobot().getY(), kEpsilon);
    assertEquals(-2.0 * flyTime, aimSolution.leadCompensation().getX(), kEpsilon);
    assertEquals(flyTime, aimSolution.leadCompensation().getY(), kEpsilon);
    assertEquals(Math.atan2(expectedCompensatedY, expectedCompensatedX), aimSolution.turretAngle(), kEpsilon);
  }

  @Test
  void CalculateAimSolutionTransformsFieldVelocityIntoRobotFrame() {
    Pose2d selfPose = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90.0));
    Translation2d targetGlobal = new Translation2d(0.0, 5.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);

    ContinuousAimCommand.AimSolution aimSolution = ContinuousAimCommand.CalculateAimSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    double flyTime = ShooterConstants.DistanceFromTargetToTime(5.0);

    assertEquals(5.0, aimSolution.targetInRobotFrame().getX(), kEpsilon);
    assertEquals(0.0, aimSolution.targetInRobotFrame().getY(), kEpsilon);
    assertEquals(5.0, aimSolution.distanceToTarget(), kEpsilon);
    assertEquals(flyTime, aimSolution.flyTime(), kEpsilon);
    assertEquals(5.0, aimSolution.compensatedTargetInRobot().getX(), kEpsilon);
    assertEquals(flyTime, aimSolution.compensatedTargetInRobot().getY(), kEpsilon);
    assertEquals(0.0, aimSolution.leadCompensation().getX(), kEpsilon);
    assertEquals(flyTime, aimSolution.leadCompensation().getY(), kEpsilon);
    assertEquals(Math.atan2(flyTime, 5.0), aimSolution.turretAngle(), kEpsilon);
  }

  @Test
  void CalculateAimSolutionReturnsZeroVectorsWhenRobotAndTargetCoincideAndStationary() {
    Pose2d selfPose = new Pose2d(new Translation2d(2.0, -1.0), Rotation2d.fromDegrees(45.0));
    Translation2d targetGlobal = new Translation2d(2.0, -1.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds();

    ContinuousAimCommand.AimSolution aimSolution = ContinuousAimCommand.CalculateAimSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    assertEquals(0.0, aimSolution.targetInRobotFrame().getX(), kEpsilon);
    assertEquals(0.0, aimSolution.targetInRobotFrame().getY(), kEpsilon);
    assertEquals(0.0, aimSolution.distanceToTarget(), kEpsilon);
    assertEquals(ShooterConstants.DistanceFromTargetToTime(0.0), aimSolution.flyTime(), kEpsilon);
    assertEquals(0.0, aimSolution.compensatedTargetInRobot().getX(), kEpsilon);
    assertEquals(0.0, aimSolution.compensatedTargetInRobot().getY(), kEpsilon);
    assertEquals(0.0, aimSolution.leadCompensation().getX(), kEpsilon);
    assertEquals(0.0, aimSolution.leadCompensation().getY(), kEpsilon);
    assertEquals(0.0, aimSolution.turretAngle(), kEpsilon);
  }

  @Test
  void CalculateAimSolutionUsesInterceptFlightTimeWhenTargetIsCoincidentButRobotIsMoving() {
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d();
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);

    ContinuousAimCommand.AimSolution aimSolution = ContinuousAimCommand.CalculateAimSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    double flyTime = ShooterConstants.DistanceFromTargetToTime(0.0);

    assertEquals(0.0, aimSolution.targetInRobotFrame().getX(), kEpsilon);
    assertEquals(0.0, aimSolution.targetInRobotFrame().getY(), kEpsilon);
    assertEquals(0.0, aimSolution.distanceToTarget(), kEpsilon);
    assertEquals(flyTime, aimSolution.flyTime(), kEpsilon);
    assertEquals(-flyTime, aimSolution.compensatedTargetInRobot().getX(), kEpsilon);
    assertEquals(0.0, aimSolution.compensatedTargetInRobot().getY(), kEpsilon);
    assertEquals(-flyTime, aimSolution.leadCompensation().getX(), kEpsilon);
    assertEquals(0.0, aimSolution.leadCompensation().getY(), kEpsilon);
    assertEquals(Math.PI, aimSolution.turretAngle(), kEpsilon);
  }

  @Test
  void CalculateAimSolutionHandlesTargetsBehindRobot() {
    Pose2d selfPose = new Pose2d(new Translation2d(), new Rotation2d());
    Translation2d targetGlobal = new Translation2d(-2.0, -3.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds();

    ContinuousAimCommand.AimSolution aimSolution = ContinuousAimCommand.CalculateAimSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    assertEquals(-2.0, aimSolution.targetInRobotFrame().getX(), kEpsilon);
    assertEquals(-3.0, aimSolution.targetInRobotFrame().getY(), kEpsilon);
    assertEquals(Math.hypot(2.0, 3.0), aimSolution.distanceToTarget(), kEpsilon);
    assertEquals(-2.0, aimSolution.compensatedTargetInRobot().getX(), kEpsilon);
    assertEquals(-3.0, aimSolution.compensatedTargetInRobot().getY(), kEpsilon);
    assertEquals(0.0, aimSolution.leadCompensation().getX(), kEpsilon);
    assertEquals(0.0, aimSolution.leadCompensation().getY(), kEpsilon);
    assertEquals(Math.atan2(-3.0, -2.0), aimSolution.turretAngle(), kEpsilon);
  }

  @Test
  void CalculateAimSolutionMatchesShooterMathForSameInputs() {
    Pose2d selfPose = new Pose2d(new Translation2d(1.0, -2.0), Rotation2d.fromDegrees(30.0));
    Translation2d targetGlobal = new Translation2d(4.0, 3.0);
    ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds(0.8, -1.2, 0.0);

    ContinuousAimCommand.AimSolution aimSolution = ContinuousAimCommand.CalculateAimSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);
    ContinuousShooter.ShotSolution shotSolution = ContinuousShooter.CalculateShotSolution(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    assertEquals(shotSolution.rawDistance(), aimSolution.distanceToTarget(), kEpsilon);
    assertEquals(shotSolution.compensatedTargetRelative().getX(), aimSolution.compensatedTargetInRobot().getX(),
        kEpsilon);
    assertEquals(shotSolution.compensatedTargetRelative().getY(), aimSolution.compensatedTargetInRobot().getY(),
        kEpsilon);
  }
}
