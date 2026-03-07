package frc.robot.command.intake;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
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
    timer.start();
  }

  @Override
  public void execute() {
    boolean joystickValue = joystickSupplier.get();
    WristRaiseLocation raiseLocation = getRaiseLocation(joystickValue);
    m_intakeSubsystem.setWristPosition(raiseLocation);

    if (raiseLocation == WristRaiseLocation.BOTTOM) {
      m_intakeSubsystem
          .runIntakeMotor(
              extakeOverrideSupplier.get() ? IntakeConstants.extakeMotorSpeed : IntakeConstants.intakeMotorSpeed);
    } else {
      m_intakeSubsystem.stopIntakeMotor();
    }
  }

  private WristRaiseLocation getRaiseLocation(boolean joystickValue) {
    if (joystickValue) {
      timer.reset();
      timer.start();
    } else {
      timer.stop();
    }

    if (timer.get() > 0.5 && timer.get() < 1.0) {
      return WristRaiseLocation.MIDDLE;
    } else if (timer.get() > 1.0) {
      return WristRaiseLocation.TOP;
    }

    return WristRaiseLocation.BOTTOM;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
