package frc.robot.command.testing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.IndexSubsystem;

public class IndexCommand extends Command {
  private final IndexSubsystem m_indexSubsystem;
  private final double m_speed;

  public IndexCommand(IndexSubsystem baseSubsystem, double speed) {
    m_indexSubsystem = baseSubsystem;
    m_speed = speed;
    addRequirements(m_indexSubsystem);
  }

  @Override
  public void execute() {
    m_indexSubsystem.runMotor(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_indexSubsystem.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
