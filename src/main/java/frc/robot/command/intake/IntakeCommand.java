package frc.robot.command.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.IntakeConstants;
import frc.robot.constant.IntakeConstants.WristRaiseLocation;
import frc.robot.subsystem.IntakeSubsystem;

public class IntakeCommand extends Command {

  private final IntakeSubsystem m_intakeSubsystem;

  private final Supplier<Boolean> joystickSupplier;
  private final Supplier<Boolean> extakeOverrideSupplier;

  private final Timer timer;
  /**
   * True once we've started the timer for the current "released" period (boot or
   * after release).
   */
  private boolean timerStartedForRelease;

  public IntakeCommand(IntakeSubsystem baseSubsystem, Supplier<Boolean> joystickSupplier,
      Supplier<Boolean> extakeOverrideSupplier) {
    m_intakeSubsystem = baseSubsystem;
    this.joystickSupplier = joystickSupplier;
    this.extakeOverrideSupplier = extakeOverrideSupplier;
    timer = new Timer();
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.stop();
    timerStartedForRelease = false;
  }

  @Override
  public void execute() {
    boolean joystickValue = joystickSupplier.get();
    WristRaiseLocation raiseLocation = getRaiseLocation(joystickValue);
    m_intakeSubsystem.setWristPosition(raiseLocation);

    // Run intake only when operator is holding trigger (intent to intake), not just
    // when wrist is down
    if (raiseLocation == WristRaiseLocation.BOTTOM && joystickValue) {
      m_intakeSubsystem
          .runIntakeMotor(
              extakeOverrideSupplier.get() ? IntakeConstants.extakeMotorSpeed : IntakeConstants.intakeMotorSpeed);
    } else {
      m_intakeSubsystem.stopIntakeMotor();
    }
  }

  private WristRaiseLocation getRaiseLocation(boolean joystickValue) {
    if (joystickValue) {
      timerStartedForRelease = false;
      timer.stop();
      return WristRaiseLocation.BOTTOM;
    }

    // Start timer when released: either first cycle after let go, or cold start
    // (boot with trigger not pressed)
    if (!timerStartedForRelease) {
      timerStartedForRelease = true;
      timer.reset();
      timer.start();
    }

    // When trigger is not pressed (including at boot), keep wrist raised
    double elapsed = timer.get();
    /*
     * if (elapsed > 1.0) {
     * return WristRaiseLocation.TOP;
     * }
     */
    if (elapsed > 1.5) {
      return WristRaiseLocation.MIDDLE;
    }
    return WristRaiseLocation.MIDDLE;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
