package frc.robot.command.shooting;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.ShooterConstants;
import frc.robot.subsystem.IndexSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import lombok.Getter;

/**
 * Shooter command with manual speed: sets shooter velocity from a supplier
 * (e.g. joystick axis) and runs the index when the shooter is up to speed.
 * Does not check turret aim; use with ManualAimCommand for full manual control.
 */
public class ContinuousManualShooter extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final IndexSubsystem indexSubsystem;
  private final Supplier<AngularVelocity> speedSupplier;
  private final BooleanSupplier indexExtakeOverrideSupplier;

  @Getter
  private static boolean isShooting = false;

  public ContinuousManualShooter(Supplier<AngularVelocity> speedSupplier) {
    this(speedSupplier, () -> false);
  }

  public ContinuousManualShooter(Supplier<AngularVelocity> speedSupplier, BooleanSupplier indexExtakeOverrideSupplier) {
    this.speedSupplier = speedSupplier;
    this.indexExtakeOverrideSupplier = indexExtakeOverrideSupplier;
    this.shooterSubsystem = ShooterSubsystem.GetInstance();
    this.indexSubsystem = IndexSubsystem.GetInstance();
    addRequirements(this.shooterSubsystem, this.indexSubsystem);
  }

  @Override
  public void execute() {
    Logger.recordOutput("ContinuousManualShooter/Time", System.currentTimeMillis());

    AngularVelocity speed = speedSupplier.get();
    shooterSubsystem.setShooterVelocity(speed);

    if (indexExtakeOverrideSupplier.getAsBoolean()) {
      isShooting = false;
      indexSubsystem.reverseRunMotor();
      return;
    }

    if (!shooterSubsystem.isShooterSpunUp()) {
      isShooting = false;
    }

    isShooting = true;
    indexSubsystem.runMotor();
  }

  @Override
  public void end(boolean interrupted) {
    isShooting = false;
    shooterSubsystem.runMotorBaseSpeed();
    indexSubsystem.stopMotor();
  }

  public static Supplier<AngularVelocity> GetBaseSpeedSupplier(Supplier<Double> sliderSupplier) {
    return () -> {
      double sliderRaw = sliderSupplier.get();
      double slider = MathUtil.clamp((sliderRaw + 1.0) / 2.0, 0.0, 1.0);
      double rps = MathUtil.interpolate(
          ShooterConstants.kShooterMinVelocity.in(Units.RotationsPerSecond),
          ShooterConstants.kShooterMaxVelocity.in(Units.RotationsPerSecond),
          slider);
      return Units.RotationsPerSecond.of(rps);
    };
  }

  public static Supplier<AngularVelocity> GetHeldSpeedSupplier(
      BooleanSupplier increaseSpeedSupplier,
      BooleanSupplier decreaseSpeedSupplier,
      AngularVelocity initialSpeed,
      double velocityRateRpmPerSecond) {
    return new Supplier<AngularVelocity>() {
      private double targetVelocityRpm = MathUtil.clamp(
          initialSpeed.in(Units.RPM),
          ShooterConstants.kShooterMinVelocity.in(Units.RPM),
          ShooterConstants.kShooterMaxVelocity.in(Units.RPM));
      private double lastTimestampSeconds = Timer.getFPGATimestamp();

      @Override
      public AngularVelocity get() {
        double currentTimestampSeconds = Timer.getFPGATimestamp();
        double deltaTimeSeconds = Math.max(0.0, currentTimestampSeconds - lastTimestampSeconds);
        lastTimestampSeconds = currentTimestampSeconds;

        double direction = 0.0;
        if (increaseSpeedSupplier.getAsBoolean()) {
          direction += 1.0;
        }
        if (decreaseSpeedSupplier.getAsBoolean()) {
          direction -= 1.0;
        }

        targetVelocityRpm = MathUtil.clamp(
            targetVelocityRpm + direction * velocityRateRpmPerSecond * deltaTimeSeconds,
            ShooterConstants.kShooterMinVelocity.in(Units.RPM),
            ShooterConstants.kShooterMaxVelocity.in(Units.RPM));
        return Units.RPM.of(targetVelocityRpm);
      }
    };
  }
}
