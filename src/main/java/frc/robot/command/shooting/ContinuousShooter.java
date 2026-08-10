package frc.robot.command.shooting;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.command.scoring.ContinuousAimCommand;
import frc.robot.constant.ShooterConstants;
import frc.robot.constant.TurretConstants;
import frc.robot.subsystem.GlobalPosition;
import frc.robot.subsystem.GlobalPosition.GMFrame;
import frc.robot.subsystem.IndexSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.LocalMath;
import lombok.Getter;

public class ContinuousShooter extends Command {
  public record ShotSolution(
      Translation2d targetRelative,
      Translation2d compensatedTargetRelative,
      double rawDistance,
      double compensatedDistance) {
  }

  private final Supplier<Translation2d> targetGlobalPoseSupplier;
  private final Supplier<Translation2d> selfGlobalPoseSupplier;
  private final BooleanSupplier indexExtakeOverrideSupplier;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final IndexSubsystem indexSubsystem;

  @Getter
  private static boolean isShooting = false;

  public ContinuousShooter(Supplier<Translation2d> targetGlobalPoseSupplier,
      Supplier<Translation2d> selfGlobalPoseSupplier,
      BooleanSupplier indexExtakeOverrideSupplier) {
    this.targetGlobalPoseSupplier = targetGlobalPoseSupplier;
    this.selfGlobalPoseSupplier = selfGlobalPoseSupplier;
    this.indexExtakeOverrideSupplier = indexExtakeOverrideSupplier;
    this.shooterSubsystem = ShooterSubsystem.GetInstance();
    this.turretSubsystem = TurretSubsystem.GetInstance();
    this.indexSubsystem = IndexSubsystem.GetInstance();

    addRequirements(this.shooterSubsystem, this.indexSubsystem);
  }

  public ContinuousShooter(Supplier<Translation2d> targetGlobalPoseSupplier,
      Supplier<Translation2d> selfGlobalPoseSupplier) {
    this(targetGlobalPoseSupplier, selfGlobalPoseSupplier, () -> false);
  }

  public ContinuousShooter(Supplier<Translation2d> targetGlobalPoseSupplier,
      BooleanSupplier indexExtakeOverrideSupplier) {
    this(targetGlobalPoseSupplier, () -> {
      return GlobalPosition.Get().getTranslation();
    }, indexExtakeOverrideSupplier);
  }

  public ContinuousShooter(Supplier<Translation2d> targetGlobalPoseSupplier) {
    this(targetGlobalPoseSupplier, () -> false);
  }

  public ContinuousShooter() {
    this(() -> new Translation2d());
  }

  public static ShotSolution CalculateShotSolution(
      Pose2d selfPose,
      Translation2d targetGlobal,
      ChassisSpeeds robotFieldSpeeds) {
    Translation2d targetRelative = LocalMath.fromGlobalToRelative(selfPose.getTranslation(), targetGlobal);
    Translation2d compensatedTargetRelative = ContinuousAimCommand.GetCompensatedSpeed(
        selfPose,
        targetGlobal,
        robotFieldSpeeds);

    double rawDistance = targetRelative.getNorm();
    double compensatedDistance = compensatedTargetRelative.getNorm();

    return new ShotSolution(targetRelative, compensatedTargetRelative, rawDistance, compensatedDistance);
  }

  @Override
  public void execute() {
    Logger.recordOutput("ContinuousShooter/Time", System.currentTimeMillis());
    Translation2d target = targetGlobalPoseSupplier.get();
    Translation2d self = selfGlobalPoseSupplier.get();
    Pose2d selfPose = new Pose2d(self, GlobalPosition.Get().getRotation());
    ChassisSpeeds robotFieldSpeeds = GlobalPosition.Velocity(GMFrame.kFieldRelative);
    ShotSolution shotSolution = CalculateShotSolution(
        selfPose,
        target,
        robotFieldSpeeds);
    shooterSubsystem.setShooterVelocity(
        ShooterConstants.DistanceFromTargetToVelocity(shotSolution.compensatedDistance()));

    Logger.recordOutput("ContinuousShooter/TargetRelative", shotSolution.targetRelative());
    Logger.recordOutput("ContinuousShooter/CompensatedTargetRelative", shotSolution.compensatedTargetRelative());
    Logger.recordOutput("ContinuousShooter/RawDistanceToTarget", shotSolution.rawDistance());
    Logger.recordOutput("ContinuousShooter/CompensatedDistanceToTarget", shotSolution.compensatedDistance());

    if (indexExtakeOverrideSupplier.getAsBoolean()) {
      isShooting = false;
      indexSubsystem.reverseRunMotor();
      return;
    }

    if (turretSubsystem.getAimTimeLeftMs() > TurretConstants.kTurretOffByMs
        || !shooterSubsystem.isShooterSpunUp()) {
      isShooting = false;
      indexSubsystem.stopMotor();
      return;
    }

    isShooting = true;
    indexSubsystem.runMotor();
  }

  @Override
  public void end(boolean interrupted) {
    isShooting = false;
    indexSubsystem.stopMotor();
  }
}
