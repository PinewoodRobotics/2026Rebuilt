package frc.robot.command.testing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.IntakeSubsystem;

public class IntakeCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final double m_speed;

  public IntakeCommand(IntakeSubsystem baseSubsystem, double speed) {
    m_intakeSubsystem = baseSubsystem;
    m_speed = speed;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void execute() {
    m_intakeSubsystem.runMotor(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
