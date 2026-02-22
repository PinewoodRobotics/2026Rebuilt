package frc.robot.command.shooting;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.ShooterConstants;
import frc.robot.subsystem.LEDSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import pwrup.frc.core.controller.FlightStick;

/**
 * Infinitely-running command that reads the speed slider and updates the
 * shooter speed display.
 * Uses LEDSubsystem so it runs alongside ShooterCommand without conflict.
 */
public class ShooterSpeedCommand extends Command {
  private static final int LED_START = 8;
  private static final int LED_END = 67;

  private static AngularVelocity targetShooterSpeed = ShooterConstants.kShooterMinVelocity;

  private final LEDSubsystem ledSubsystem;
  private final FlightStick flightStick;

  public ShooterSpeedCommand(LEDSubsystem ledSubsystem, FlightStick flightStick) {
    this.ledSubsystem = ledSubsystem;
    this.flightStick = flightStick;
    addRequirements(ledSubsystem);
  }

  private static final String BAR_ID = "ShooterSpeedPercent";
  private static final String OVERLAY_ID = "ShooterActualPercent";

  @Override
  public void initialize() {
    ledSubsystem.addProgressBar(BAR_ID, LED_START, LED_END,
        new RGBWColor(0, 255, 0), new RGBWColor(15, 15, 15));
    // This overlays actual speed and is additively blended in LEDSubsystem.
    ledSubsystem.addProgressBarOverlay(OVERLAY_ID, LED_START, LED_END, new RGBWColor(100, 0, 255));
  }

  @Override
  public void execute() {
    double percent = Math.max(0, Math.min(1, (flightStick.getRightSlider() + 1) / 2));
    double min = ShooterConstants.kShooterMinVelocity.in(Units.RotationsPerSecond);
    double max = ShooterConstants.kShooterMaxVelocity.in(Units.RotationsPerSecond);
    targetShooterSpeed = Units.RotationsPerSecond.of(min + (max - min) * percent);

    ledSubsystem.setProgress(BAR_ID, percent);
    double actualPercent = ShooterSubsystem.GetInstance().getCurrentShooterVelocity()
        .in(Units.RotationsPerSecond) / ShooterConstants.kShooterMaxVelocity.in(Units.RotationsPerSecond);
    ledSubsystem.setProgress(OVERLAY_ID, Math.max(0, Math.min(1, actualPercent)));

    Logger.recordOutput("Shooter/NeededSpeed", targetShooterSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static AngularVelocity getTargetShooterSpeed() {
    return targetShooterSpeed;
  }
}
