package frc.robot.command.util;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Default command for {@link frc.robot.subsystem.LightsSubsystem} that runs a
 * list of sub-commands every cycle, simulating scheduler lifecycle: initialize
 * once per sub-command, execute until finished, end on finish or when this
 * command is interrupted.
 */
public class PollingCommand extends Command {

  private final Supplier<List<Command>> commandSupplier;
  private final Set<Integer> initializedCommandIds = new HashSet<>();

  public PollingCommand(Subsystem subsystem, Supplier<List<Command>> commandSupplier) {
    this.commandSupplier = commandSupplier;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // Sub-commands are lazily initialized on first execute when they appear in the
    // list
  }

  @Override
  public void execute() {
    for (Command command : commandSupplier.get()) {
      int commandId = command.getName().hashCode();
      if (!initializedCommandIds.contains(commandId) && !command.isFinished()) {
        command.initialize();
        initializedCommandIds.add(commandId);

      }

      if (command.isFinished()) {
        command.end(false);
        initializedCommandIds.remove(commandId);
      } else {
        command.execute();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    for (Command command : commandSupplier.get()) {
      if (initializedCommandIds.contains(command.getName().hashCode())) {
        command.end(interrupted);
      }
    }

    initializedCommandIds.clear();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isCommandAlreadyInserted(Command command) {
    for (Command existing : commandSupplier.get()) {
      if (existing.getName().equals(command.getName())) {
        return true;
      }
    }
    return false;
  }
}
