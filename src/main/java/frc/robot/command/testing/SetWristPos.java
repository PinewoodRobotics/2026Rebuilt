package frc.robot.command.testing;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.IntakeSubsystem;

public class SetWristPos extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final Rotation2d m_targetPosition;

  public SetWristPos(IntakeSubsystem baseSubsystem, Rotation2d targetPosition) {
    m_intakeSubsystem = baseSubsystem;
    m_targetPosition = targetPosition;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.setWristPosition(m_targetPosition);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
