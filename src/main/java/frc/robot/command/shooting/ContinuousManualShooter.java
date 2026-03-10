package frc.robot.command.shooting;

import java.util.function.Supplier;

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

  @Getter
  private static boolean isShooting = false;

  public ContinuousManualShooter(Supplier<AngularVelocity> speedSupplier) {
    this.speedSupplier = speedSupplier;
    this.shooterSubsystem = ShooterSubsystem.GetInstance();
    this.indexSubsystem = IndexSubsystem.GetInstance();
    addRequirements(this.shooterSubsystem, this.indexSubsystem);
  }

  @Override
  public void execute() {
    AngularVelocity speed = speedSupplier.get();
    shooterSubsystem.setShooterVelocity(speed);

    if (shooterSubsystem.timeLeftToReachVelocity() > ShooterConstants.kShooterOffByMs) {
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
    shooterSubsystem.runMotorBaseSpeed();
    indexSubsystem.stopMotor();
  }
}
