package frc.robot.hardware;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constant.HardwareConstants;
import frc.robot.subsystem.GlobalPosition;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;

public class UnifiedGyro extends SubsystemBase implements IGyroscopeLike {
  private static UnifiedGyro instance;

  private final List<IGyroscopeLike> gyros;

  public UnifiedGyro(IGyroscopeLike... gyros) {
    this.gyros = Arrays.asList(gyros);
  }

  public static UnifiedGyro GetInstance() {
    if (instance == null) {
      instance = new UnifiedGyro(Arrays.stream(HardwareConstants.kPigeonConfigs)
          .map(PigeonGyro::GetInstance)
          .toArray(IGyroscopeLike[]::new));
    }

    return instance;
  }

  private IGyroscopeLike getGlobalRotationClosest(Pose2d pose) {
    if (gyros.isEmpty()) {
      throw new IllegalStateException("No gyros configured");
    }

    if (pose == null || gyros.size() == 1) {
      return gyros.get(0);
    }

    var targetYaw = pose.getRotation();
    IGyroscopeLike closestGyro = gyros.get(0);
    double smallestError = Math.abs(closestGyro.getRotation().toRotation2d().minus(targetYaw).getRadians());

    for (int i = 1; i < gyros.size(); i++) {
      var gyro = gyros.get(i);
      double error = Math.abs(gyro.getRotation().toRotation2d().minus(targetYaw).getRadians());
      if (error < smallestError) {
        smallestError = error;
        closestGyro = gyro;
      }
    }

    return closestGyro;
  }

  private IGyroscopeLike getMainGyro() {
    if (HardwareConstants.kRobotMainGyro == HardwareConstants.RobotMainGyro.GlobalClosest
        && !GlobalPosition.isValid()) {
      return gyros.get(0);
    }

    switch (HardwareConstants.kRobotMainGyro) {
      case GlobalClosest:
        return getGlobalRotationClosest(GlobalPosition.Get());
      case One:
        return gyros.get(0);
      case Two:
        return gyros.get(1);
      default:
        throw new IllegalArgumentException("Invalid robot main gyro: " + HardwareConstants.kRobotMainGyro);
    }
  }

  @Override
  public ChassisSpeeds getVelocity() {
    return getMainGyro().getVelocity();
  }

  @Override
  public ChassisSpeeds getAcceleration() {
    return getMainGyro().getAcceleration();
  }

  @Override
  public Rotation3d getRotation() {
    return getMainGyro().getRotation();
  }

  @Override
  public void resetRotation(Rotation3d newRotation) {
    for (IGyroscopeLike gyro : gyros) {
      gyro.resetRotation(newRotation);
    }
  }
}
