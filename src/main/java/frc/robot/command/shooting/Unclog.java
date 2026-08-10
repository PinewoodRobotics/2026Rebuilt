package frc.robot.command.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.IndexSubsystem;

public class Unclog extends Command {
  private final IndexSubsystem indexSubsystem;
  private static final double unclogTimeSeconds = 1.0;
  private final Timer timer;

  public Unclog() {
    this.indexSubsystem = IndexSubsystem.GetInstance();
    this.timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    indexSubsystem.reverseRunMotor();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(unclogTimeSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    indexSubsystem.stopMotor();
    timer.stop();
  }
}
