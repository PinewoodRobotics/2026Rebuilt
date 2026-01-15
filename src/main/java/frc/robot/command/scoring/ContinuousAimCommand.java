package frc.robot.command.scoring;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.LocalMath;

public class ContinuousAimCommand extends Command {
    private final TurretSubsystem turretSubsystem;
    private final Supplier<Translation3d> targetGlobalPoseSupplier;
    private final Supplier<Pose2d> selfGlobalPoseSupplier;
    private final Supplier<Translation2d> currentRobotVelocitySupplier;

    public ContinuousAimCommand(Supplier<Translation3d> targetGlobalPoseSupplier,
            Supplier<Pose2d> selfGlobalPoseSupplier,
            Supplier<Translation2d> currentRobotVelocitySupplier) {
        this.turretSubsystem = TurretSubsystem.GetInstance();
        this.targetGlobalPoseSupplier = targetGlobalPoseSupplier;
        this.selfGlobalPoseSupplier = selfGlobalPoseSupplier;
        this.currentRobotVelocitySupplier = currentRobotVelocitySupplier;

        addRequirements(this.turretSubsystem);
    }

    public ContinuousAimCommand(Supplier<Translation3d> targetGlobalPoseSupplier,
            Supplier<Pose2d> selfGlobalPoseSupplier) {
        this(targetGlobalPoseSupplier, selfGlobalPoseSupplier, () -> new Translation2d(0, 0));
    }

    @Override
    public void execute() {
        Pose2d selfPose = selfGlobalPoseSupplier.get();
        Translation3d targetGlobal = targetGlobalPoseSupplier.get();

        Translation2d selfTranslation = selfPose.getTranslation();
        Translation2d targetTranslation = targetGlobal.toTranslation2d();

        Translation2d target = LocalMath.fromGlobalToRelative(selfTranslation, targetTranslation);
        Translation2d velocity = currentRobotVelocitySupplier.get();

        Translation2d aimPoint = target.minus(velocity);

        Angle newAngle = Angle.ofRelativeUnits(aimPoint.getAngle().getRadians(), Units.Radians);

        turretSubsystem.setTurretPosition(newAngle);

        logEverything(selfPose, targetGlobal, target, velocity, aimPoint, newAngle);
    }

    private void logEverything(
            Pose2d selfPose,
            Translation3d targetGlobal,
            Translation2d targetRelative,
            Translation2d robotVelocity,
            Translation2d aimPoint,
            Angle commandedAngle) {
        // Raw inputs
        Logger.recordOutput("Turret/AimCommand/SelfPose", selfPose);
        Logger.recordOutput("Turret/AimCommand/TargetGlobal", targetGlobal);
        Logger.recordOutput("Turret/AimCommand/RobotVelocity", robotVelocity);

        // Derived math
        Logger.recordOutput("Turret/AimCommand/TargetRelative", targetRelative);
        Logger.recordOutput("Turret/AimCommand/TargetDistanceMeters", targetRelative.getNorm());
        Logger.recordOutput("Turret/AimCommand/AimPoint", aimPoint);
        Logger.recordOutput("Turret/AimCommand/AimDistanceMeters", aimPoint.getNorm());

        // Commanded angle
        double goalRad = commandedAngle.in(Units.Radians);
        Logger.recordOutput("Turret/AimCommand/GoalAngleRad", goalRad);
        Logger.recordOutput("Turret/AimCommand/GoalAngleDeg", commandedAngle.in(Units.Degrees));

        // Turret feedback vs goal
        double currentRad = turretSubsystem.getTurretPosition().in(Units.Radians);
        double errorRad = Math.IEEEremainder(goalRad - currentRad, 2.0 * Math.PI); // wrap to [-pi, pi]
        Logger.recordOutput("Turret/AimCommand/CurrentAngleRad", currentRad);
        Logger.recordOutput("Turret/AimCommand/ErrorRad", errorRad);
        Logger.recordOutput("Turret/AimCommand/ErrorDeg", Math.toDegrees(errorRad));

        // Timing (kept for compatibility with existing dashboards)
        Logger.recordOutput("Turret/TimeLeftToReachPosition", turretSubsystem.getAimTimeLeftMs());
    }
}
