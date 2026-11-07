package frc.robot.command.shooting;

import org.wpilib.units.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.command2.Command;
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
