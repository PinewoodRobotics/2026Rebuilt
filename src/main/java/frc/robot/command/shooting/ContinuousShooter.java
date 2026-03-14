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

  public ContinuousShooter(Supplier<Translation2d> targetGlobalPoseSupplier, BooleanSupplier indexExtakeOverrideSupplier) {
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

  @Override
  public void execute() {
    Logger.recordOutput("ContinuousShooter/Time", System.currentTimeMillis());
    Translation2d target = targetGlobalPoseSupplier.get();
    Translation2d self = selfGlobalPoseSupplier.get();
    Translation2d targetRelative = LocalMath.fromGlobalToRelative(self, target);
    Pose2d selfPose = new Pose2d(self, GlobalPosition.Get().getRotation());
    ChassisSpeeds robotFieldSpeeds = GlobalPosition.Velocity(GMFrame.kFieldRelative);
    Translation2d compensatedTargetRelative = ContinuousAimCommand.GetCompensatedSpeed(
        selfPose,
        target,
        robotFieldSpeeds);

    double rawDistance = targetRelative.getNorm();
    double compensatedDistance = compensatedTargetRelative.getNorm();
    shooterSubsystem.setShooterVelocity(
        ShooterConstants.DistanceFromTargetToVelocity(compensatedDistance));

    Logger.recordOutput("ContinuousShooter/TargetRelative", targetRelative);
    Logger.recordOutput("ContinuousShooter/CompensatedTargetRelative", compensatedTargetRelative);
    Logger.recordOutput("ContinuousShooter/RawDistanceToTarget", rawDistance);
    Logger.recordOutput("ContinuousShooter/CompensatedDistanceToTarget", compensatedDistance);

    if (indexExtakeOverrideSupplier.getAsBoolean()) {
      isShooting = false;
      indexSubsystem.reverseRunMotor();
      return;
    }

    if (turretSubsystem.getAimTimeLeftMs() > TurretConstants.kTurretOffByMs
        || !shooterSubsystem.isShooterSpunUp()) {
      isShooting = false;
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
