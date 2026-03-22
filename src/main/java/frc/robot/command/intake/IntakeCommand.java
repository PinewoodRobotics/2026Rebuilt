package frc.robot.command.intake;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.IntakeConstants;
import frc.robot.constant.IntakeConstants.WristRaiseLocation;
import frc.robot.subsystem.IntakeSubsystem;

public class IntakeCommand extends Command {
  private static final double kExtakePulseDurationS = 0.5;
  private static final double kExtakePulsePeriodS = 1.0;

  private final IntakeSubsystem m_intakeSubsystem;

  private final Supplier<Boolean> joystickSupplier;
  private final Supplier<Boolean> extakeOverrideSupplier;
  private WristRaiseLocation alternateRaiseLocation = WristRaiseLocation.MIDDLE;

  private boolean wasRunningIntake;
  private double intakeCycleStartS;

  public void setAlternateRaiseLocation(WristRaiseLocation location) {
    alternateRaiseLocation = location;
  }

  public IntakeCommand(IntakeSubsystem baseSubsystem, Supplier<Boolean> joystickSupplier,
      Supplier<Boolean> extakeOverrideSupplier, WristRaiseLocation raiseLocation) {
    this.m_intakeSubsystem = baseSubsystem;
    this.joystickSupplier = joystickSupplier;
    this.extakeOverrideSupplier = extakeOverrideSupplier;
    this.alternateRaiseLocation = raiseLocation;
    this.wasRunningIntake = false;
    this.intakeCycleStartS = 0.0;

    addRequirements(m_intakeSubsystem);
  }

  public IntakeCommand(IntakeSubsystem baseSubsystem, Supplier<Boolean> joystickSupplier,
      Supplier<Boolean> extakeOverrideSupplier) {
    this(baseSubsystem, joystickSupplier, extakeOverrideSupplier, WristRaiseLocation.MIDDLE);
  }

  @Override
  public void initialize() {
    wasRunningIntake = false;
    intakeCycleStartS = 0.0;
  }

  @Override
  public void execute() {
    if (alternateRaiseLocation == null) {
      alternateRaiseLocation = WristRaiseLocation.BOTTOM;
    }

    boolean joystickValue = joystickSupplier.get();
    boolean isExtake = extakeOverrideSupplier.get();
    if (joystickValue) {
      if (!wasRunningIntake) {
        intakeCycleStartS = Timer.getFPGATimestamp();
        wasRunningIntake = true;
      }

      double cycleElapsedS = Timer.getFPGATimestamp() - intakeCycleStartS;
      boolean shouldPulseExtake = (cycleElapsedS % kExtakePulsePeriodS) < kExtakePulseDurationS;

      m_intakeSubsystem.setWristPosition(WristRaiseLocation.BOTTOM);
      if (isExtake) {
        m_intakeSubsystem.runIntakeMotor(IntakeConstants.extakeMotorSpeed);
      } else if (IntakeConstants.kShouldWeAreWeGoingToPulseTheIntakeConstantVariable) {
        m_intakeSubsystem.runIntakeMotor(0);
      } else {
        m_intakeSubsystem.runIntakeMotor(IntakeConstants.intakeMotorSpeed);
      }

      // m_indexSubsystem.runMotor(isExtake ? -IndexConstants.kIndexMotorSpeed :
      // IndexConstants.kIndexMotorSpeed);
    } else {
      wasRunningIntake = false;
      intakeCycleStartS = 0.0;
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
