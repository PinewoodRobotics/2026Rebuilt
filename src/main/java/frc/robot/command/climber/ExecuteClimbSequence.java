package frc.robot.command.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ClimberSubsystem;

public class ExecuteClimbSequence extends Command {

  private final ClimberSubsystem climberSubsystem;

  public ExecuteClimbSequence(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }
}
