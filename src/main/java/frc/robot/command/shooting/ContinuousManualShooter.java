package frc.robot.command.shooting;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.ShooterConstants;
import frc.robot.subsystem.IndexSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import lombok.Getter;

/**
 * Shooter command with manual speed: sets shooter velocity from a supplier
 * (e.g. joystick axis) and runs the index when the shooter is up to speed.
 * Does not check turret aim; use with ManualAimCommand for full manual control.
 */
public class ContinuousManualShooter extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final IndexSubsystem indexSubsystem;
  private final Supplier<AngularVelocity> speedSupplier;
  private final BooleanSupplier indexExtakeOverrideSupplier;

  @Getter
  private static boolean isShooting = false;

  public ContinuousManualShooter(Supplier<AngularVelocity> speedSupplier) {
    this(speedSupplier, () -> false);
  }

  public ContinuousManualShooter(Supplier<AngularVelocity> speedSupplier, BooleanSupplier indexExtakeOverrideSupplier) {
    this.speedSupplier = speedSupplier;
    this.indexExtakeOverrideSupplier = indexExtakeOverrideSupplier;
    this.shooterSubsystem = ShooterSubsystem.GetInstance();
    this.indexSubsystem = IndexSubsystem.GetInstance();
    addRequirements(this.shooterSubsystem, this.indexSubsystem);
  }

  @Override
  public void execute() {
    Logger.recordOutput("ContinuousManualShooter/Time", System.currentTimeMillis());
    AngularVelocity speed = speedSupplier.get();
    shooterSubsystem.setShooterVelocity(speed);

    if (indexExtakeOverrideSupplier.getAsBoolean()) {
      isShooting = false;
      indexSubsystem.reverseRunMotor();
      return;
    }

    if (!shooterSubsystem.isShooterSpunUp()) {
      isShooting = false;
    }

    isShooting = true;
    indexSubsystem.runMotor();
  }

  @Override
  public void end(boolean interrupted) {
    isShooting = false;
    shooterSubsystem.runMotorBaseSpeed();
    indexSubsystem.stopMotor();
  }

  public static Supplier<AngularVelocity> GetBaseSpeedSupplier(Supplier<Double> sliderSupplier) {
    return () -> {
      double sliderRaw = sliderSupplier.get();
      double slider = MathUtil.clamp((sliderRaw + 1.0) / 2.0, 0.0, 1.0);
      double rps = MathUtil.interpolate(
          ShooterConstants.kShooterMinVelocity.in(Units.RotationsPerSecond),
          ShooterConstants.kShooterMaxVelocity.in(Units.RotationsPerSecond),
          slider);
      return Units.RotationsPerSecond.of(rps);
    };
  }
}
