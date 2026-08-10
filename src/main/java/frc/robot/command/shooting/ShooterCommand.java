package frc.robot.command.shooting;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;

import java.util.function.Supplier;

public class ShooterCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final Supplier<AngularVelocity> speed;

  public ShooterCommand(ShooterSubsystem shooterSubsystem, Supplier<AngularVelocity> speed) {
    this.shooterSubsystem = shooterSubsystem;
    this.speed = speed;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.setShooterVelocity(speed.get());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }
}
