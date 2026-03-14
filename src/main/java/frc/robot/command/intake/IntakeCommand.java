package frc.robot.command.intake;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.IntakeConstants;
import frc.robot.constant.IntakeConstants.WristRaiseLocation;
import frc.robot.subsystem.IntakeSubsystem;

public class IntakeCommand extends Command {

  private final IntakeSubsystem m_intakeSubsystem;

  private final Supplier<Boolean> joystickSupplier;
  private final Supplier<Boolean> extakeOverrideSupplier;
  private WristRaiseLocation alternateRaiseLocation = WristRaiseLocation.MIDDLE;

  public void setAlternateRaiseLocation(WristRaiseLocation location) {
    alternateRaiseLocation = location;
  }

  public IntakeCommand(IntakeSubsystem baseSubsystem, Supplier<Boolean> joystickSupplier,
      Supplier<Boolean> extakeOverrideSupplier, WristRaiseLocation raiseLocation) {
    m_intakeSubsystem = baseSubsystem;
    this.joystickSupplier = joystickSupplier;
    this.extakeOverrideSupplier = extakeOverrideSupplier;

    alternateRaiseLocation = raiseLocation;

    addRequirements(m_intakeSubsystem);
  }

  public IntakeCommand(IntakeSubsystem baseSubsystem, Supplier<Boolean> joystickSupplier,
      Supplier<Boolean> extakeOverrideSupplier) {
    this(baseSubsystem, joystickSupplier, extakeOverrideSupplier, WristRaiseLocation.MIDDLE);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    boolean joystickValue = joystickSupplier.get();
    if (joystickValue) {
      m_intakeSubsystem.setWristPosition(WristRaiseLocation.BOTTOM);
      m_intakeSubsystem
          .runIntakeMotor(
              extakeOverrideSupplier.get() ? IntakeConstants.extakeMotorSpeed : IntakeConstants.intakeMotorSpeed);

    } else {
      m_intakeSubsystem
          .setWristPosition(alternateRaiseLocation);
      m_intakeSubsystem.stopIntakeMotor();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
