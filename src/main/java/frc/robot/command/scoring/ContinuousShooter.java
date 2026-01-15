package frc.robot.command.scoring;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.TurretConstants;
import frc.robot.subsystem.GlobalPosition;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.util.CustomMath;

public class ContinuousShooter extends Command {
    private final Supplier<Translation3d> targetGlobalPoseSupplier;
    private final Supplier<Translation3d> selfGlobalPoseSupplier;

    public ContinuousShooter(Supplier<Translation3d> targetGlobalPoseSupplier,
            Supplier<Translation3d> selfGlobalPoseSupplier) {
        this.targetGlobalPoseSupplier = targetGlobalPoseSupplier;
        this.selfGlobalPoseSupplier = selfGlobalPoseSupplier;

        addRequirements(ShooterSubsystem.GetInstance());
    }

    public ContinuousShooter(Supplier<Translation3d> targetGlobalPoseSupplier) {
        this(targetGlobalPoseSupplier, () -> {
            Pose2d selfGlobalPose = GlobalPosition.Get();
            return new Translation3d(selfGlobalPose.getX(), selfGlobalPose.getY(), 0);
        });
    }

    @Override
    public void execute() {
        Translation3d targetGlobalPose = targetGlobalPoseSupplier.get();
        Translation3d selfGlobalPose = selfGlobalPoseSupplier.get();

        Translation2d target = CustomMath.fromGlobalToRelative(selfGlobalPose.toTranslation2d(),
                targetGlobalPose.toTranslation2d());

        double distance = target.getNorm();
        double height = targetGlobalPose.getZ() - selfGlobalPose.getZ();
        double theta = TurretConstants.kTurretTheta.in(Units.Radians);

        LinearVelocity speed = calculateSpeedNeeded(distance, height, theta);

        ShooterSubsystem.GetInstance().setShooterVelocity(speed);
    }

    private LinearVelocity calculateSpeedNeeded(double x, double y, double angleRad) {
        double t = Math.sqrt((x * Math.tan(angleRad) - y) / 4.9);
        double v = x / (Math.cos(angleRad) * t);
        return LinearVelocity.ofRelativeUnits(v, Units.MetersPerSecond);
    }
}
