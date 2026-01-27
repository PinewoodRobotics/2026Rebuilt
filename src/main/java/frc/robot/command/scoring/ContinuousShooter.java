package frc.robot.command.scoring;

import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.ShooterConstants;
import frc.robot.constant.TurretConstants;
import frc.robot.subsystem.GlobalPosition;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.LocalMath;

public class ContinuousShooter extends Command {
    private final Supplier<Translation3d> targetGlobalPoseSupplier;
    private final Supplier<Translation3d> selfGlobalPoseSupplier;
    private final Function<Void, Void> feedShooter;
    private final ShooterSubsystem shooterSubsystem;
    private final TurretSubsystem turretSubsystem;

    public ContinuousShooter(Supplier<Translation3d> targetGlobalPoseSupplier,
            Supplier<Translation3d> selfGlobalPoseSupplier, Function<Void, Void> feedShooter) {
        this.targetGlobalPoseSupplier = targetGlobalPoseSupplier;
        this.selfGlobalPoseSupplier = selfGlobalPoseSupplier;
        this.feedShooter = feedShooter;
        this.shooterSubsystem = ShooterSubsystem.GetInstance();
        this.turretSubsystem = TurretSubsystem.GetInstance();

        addRequirements(this.shooterSubsystem);
    }

    public ContinuousShooter(Supplier<Translation3d> targetGlobalPoseSupplier) {
        this(targetGlobalPoseSupplier, () -> {
            Pose2d selfGlobalPose = GlobalPosition.Get();
            return new Translation3d(selfGlobalPose.getX(), selfGlobalPose.getY(), 0);
        }, (Void) -> {
            return null;
        });
    }

    @Override
    public void execute() {
        Translation3d targetGlobalPose = targetGlobalPoseSupplier.get();
        Translation3d selfGlobalPose = selfGlobalPoseSupplier.get();

        Translation2d target = LocalMath.fromGlobalToRelative(selfGlobalPose.toTranslation2d(),
                targetGlobalPose.toTranslation2d());

        double distance = target.getNorm();
        double height = targetGlobalPose.getZ() - selfGlobalPose.getZ();
        double theta = TurretConstants.kTurretTheta.in(Units.Radians);

        LinearVelocity speed = calculateSpeedNeeded(distance, height, theta);

        logEverything(targetGlobalPose, selfGlobalPose, target, distance, height, theta, speed);

        shooterSubsystem.setShooterVelocity(speed);

        if (shooterSubsystem.timeLeftToReachVelocity() >= ShooterConstants.kShooterOffByMs
                || turretSubsystem.getAimTimeLeftMs() >= TurretConstants.kTurretOffByMs) {
            Logger.recordOutput("ContinuousShooter/Feeding", false);
            return;
        }

        Logger.recordOutput("ContinuousShooter/Feeding", true);
        feedShooter.apply(null);
    }

    private LinearVelocity calculateSpeedNeeded(double x, double y, double angleRad) {
        double t = Math.sqrt((x * Math.tan(angleRad) - y) / 4.9);
        double v = x / (Math.cos(angleRad) * t);
        return Units.MetersPerSecond.of(v);
    }

    private void logEverything(
            Translation3d targetGlobalPose,
            Translation3d selfGlobalPose,
            Translation2d targetRelative,
            double distanceMeters,
            double heightMeters,
            double thetaRad,
            LinearVelocity shooterSetpoint) {
        // Poses / geometry
        Logger.recordOutput("ContinuousShooter/TargetGlobalPose/X", targetGlobalPose.getX());
        Logger.recordOutput("ContinuousShooter/TargetGlobalPose/Y", targetGlobalPose.getY());
        Logger.recordOutput("ContinuousShooter/TargetGlobalPose/Z", targetGlobalPose.getZ());

        Logger.recordOutput("ContinuousShooter/SelfGlobalPose/X", selfGlobalPose.getX());
        Logger.recordOutput("ContinuousShooter/SelfGlobalPose/Y", selfGlobalPose.getY());
        Logger.recordOutput("ContinuousShooter/SelfGlobalPose/Z", selfGlobalPose.getZ());

        Logger.recordOutput("ContinuousShooter/TargetRelative/X", targetRelative.getX());
        Logger.recordOutput("ContinuousShooter/TargetRelative/Y", targetRelative.getY());
        Logger.recordOutput("ContinuousShooter/DistanceMeters", distanceMeters);
        Logger.recordOutput("ContinuousShooter/HeightMeters", heightMeters);

        Logger.recordOutput("ContinuousShooter/ThetaRad", thetaRad);
        Logger.recordOutput("ContinuousShooter/ThetaDeg", Math.toDegrees(thetaRad));

        // Shooter
        Logger.recordOutput("ContinuousShooter/ShooterSetpointMps", shooterSetpoint.in(Units.MetersPerSecond));
        Logger.recordOutput("ContinuousShooter/ShooterCurrentMps",
                shooterSubsystem.getCurrentShooterVelocity().in(Units.MetersPerSecond));
        Logger.recordOutput("ContinuousShooter/ShooterTimeLeftMs", shooterSubsystem.timeLeftToReachVelocity());
        Logger.recordOutput("ContinuousShooter/ShooterReady",
                shooterSubsystem.timeLeftToReachVelocity() < ShooterConstants.kShooterOffByMs);

        // Turret
        Logger.recordOutput("ContinuousShooter/TurretTimeLeftMs", turretSubsystem.getAimTimeLeftMs());
        Logger.recordOutput("ContinuousShooter/TurretPositionDeg",
                turretSubsystem.getTurretPosition().in(Units.Degrees));
        Logger.recordOutput("ContinuousShooter/TurretReady",
                turretSubsystem.getAimTimeLeftMs() < TurretConstants.kTurretOffByMs);
    }
}
