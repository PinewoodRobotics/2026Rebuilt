package frc.robot.command.scoring;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
        Translation2d selfTranslation = selfGlobalPoseSupplier.get().getTranslation();
        Translation2d targetTranslation = targetGlobalPoseSupplier.get().toTranslation2d();

        Translation2d target = LocalMath.fromGlobalToRelative(selfTranslation, targetTranslation);
        Translation2d velocity = currentRobotVelocitySupplier.get();

        Translation2d aimPoint = target.minus(velocity);

        Angle newAngle = Angle.ofRelativeUnits(aimPoint.getAngle().getRadians(), Units.Radians);

        turretSubsystem.setTurretPosition(newAngle);
    }

    private void logEverything() {
        Logger.recordOutput("Turret/TimeLeftToReachPosition", turretSubsystem.getAimTimeLeftMs());
    }
}
