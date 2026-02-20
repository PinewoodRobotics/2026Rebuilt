package frc.robot.command.shooting;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ShooterSubsystem;

public class ShooterCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;

  public ShooterCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.setShooterVelocity(Units.RotationsPerSecond.of(35.0));
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }
}
