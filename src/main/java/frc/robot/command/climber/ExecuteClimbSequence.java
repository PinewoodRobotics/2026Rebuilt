package frc.robot.command.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.ClimberConstants.ClimberPosition;
import frc.robot.subsystem.ClimberSubsystem;

public class ExecuteClimbSequence extends Command {

  private final ClimberSubsystem climberSubsystem;
  private final ClimberPosition[] positions;

  public ExecuteClimbSequence(ClimberSubsystem climberSubsystem, ClimberPosition[] positions) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
    this.positions = positions;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    for (ClimberPosition position : positions) {
      climberSubsystem.setHeight(position.positionMoveSequence[0]);
      climberSubsystem.setHeight(position.positionMoveSequence[1]);
    }
  }
}
