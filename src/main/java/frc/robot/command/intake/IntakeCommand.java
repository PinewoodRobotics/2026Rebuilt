package frc.robot.command.intake;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.IndexConstants;
import frc.robot.constant.IntakeConstants;
import frc.robot.constant.IntakeConstants.WristRaiseLocation;
import frc.robot.subsystem.IndexSubsystem;
import frc.robot.subsystem.IntakeSubsystem;

public class IntakeCommand extends Command {

  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexSubsystem m_indexSubsystem;

  private final Supplier<Boolean> joystickSupplier;
  private final Supplier<Boolean> extakeOverrideSupplier;
  private WristRaiseLocation alternateRaiseLocation = WristRaiseLocation.MIDDLE;

  public void setAlternateRaiseLocation(WristRaiseLocation location) {
    alternateRaiseLocation = location;
  }

  public IntakeCommand(IntakeSubsystem baseSubsystem, Supplier<Boolean> joystickSupplier,
      Supplier<Boolean> extakeOverrideSupplier, WristRaiseLocation raiseLocation) {
    m_intakeSubsystem = baseSubsystem;
    m_indexSubsystem = IndexSubsystem.GetInstance();
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
    if (alternateRaiseLocation == null) {
      alternateRaiseLocation = WristRaiseLocation.BOTTOM;
    }

    boolean joystickValue = joystickSupplier.get();
    boolean isExtake = extakeOverrideSupplier.get();
    if (joystickValue) {
      m_intakeSubsystem.setWristPosition(WristRaiseLocation.BOTTOM);
      m_intakeSubsystem
          .runIntakeMotor(
              isExtake ? IntakeConstants.extakeMotorSpeed : IntakeConstants.intakeMotorSpeed);
      // m_indexSubsystem.runMotor(isExtake ? -IndexConstants.kIndexMotorSpeed :
      // IndexConstants.kIndexMotorSpeed);
    } else {
      m_intakeSubsystem
          .setWristPosition(alternateRaiseLocation);
      m_intakeSubsystem.stopIntakeMotor();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
