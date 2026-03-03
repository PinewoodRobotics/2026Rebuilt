package frc.robot.command.util;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.LocalMath;

/**
 * Default command for {@link frc.robot.subsystem.LightsSubsystem} that runs a
 * list of sub-commands every cycle, simulating scheduler lifecycle: initialize
 * once per sub-command, execute until finished, end on finish or when this
 * command is interrupted.
 */
public class PollingCommand extends Command {
  public static abstract class IdCommand extends Command {
    private final int id;

    public IdCommand() {
      id = LocalMath.randomInt(0, 1000000);
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) {
        return true;
      }
      if (!(obj instanceof IdCommand other)) {
        return false;
      }
      return id == other.id;
    }

    @Override
    public int hashCode() {
      return id;
    }

    public int getCommandId() {
      return id;
    }
  }

  private final Supplier<List<IdCommand>> commandSupplier;
  private final Set<Integer> initializedCommandIds = new HashSet<>();

  public PollingCommand(Subsystem subsystem, Supplier<List<IdCommand>> commandSupplier) {
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
    for (IdCommand command : commandSupplier.get()) {
      int commandId = command.getCommandId();
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
    for (IdCommand command : commandSupplier.get()) {
      if (initializedCommandIds.contains(command.getCommandId())) {
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
    if (!(command instanceof IdCommand idCommand)) {
      return false;
    }
    for (IdCommand existing : commandSupplier.get()) {
      if (existing.equals(idCommand)) {
        return true;
      }
    }
    return false;
  }
}
