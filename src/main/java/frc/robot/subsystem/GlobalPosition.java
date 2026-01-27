package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GlobalPosition {
    public static Pose2d Get() {
        return new Pose2d(0, 0, new Rotation2d(0));
    }
}
