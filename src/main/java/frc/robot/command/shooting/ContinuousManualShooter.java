package frc.robot.command.shooting;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.util.MathUtil;
import org.wpilib.units.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
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
      indexSubsystem.stopMotor();
      return;
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
      double slider = Math.clamp((sliderRaw + 1.0) / 2.0, 0.0, 1.0);
      double rps = MathUtil.lerp(
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
      private double targetVelocityRpm = Math.clamp(
          initialSpeed.in(Units.RPM),
          ShooterConstants.kShooterMinVelocity.in(Units.RPM),
          ShooterConstants.kShooterMaxVelocity.in(Units.RPM));
      private double lastTimestampSeconds = Timer.getTimestamp();

      @Override
      public AngularVelocity get() {
        double currentTimestampSeconds = Timer.getTimestamp();
        double deltaTimeSeconds = Math.max(0.0, currentTimestampSeconds - lastTimestampSeconds);
        lastTimestampSeconds = currentTimestampSeconds;

        double direction = 0.0;
        if (increaseSpeedSupplier.getAsBoolean()) {
          direction += 1.0;
        }
        if (decreaseSpeedSupplier.getAsBoolean()) {
          direction -= 1.0;
        }

        targetVelocityRpm = Math.clamp(
            targetVelocityRpm + direction * velocityRateRpmPerSecond * deltaTimeSeconds,
            ShooterConstants.kShooterMinVelocity.in(Units.RPM),
            ShooterConstants.kShooterMaxVelocity.in(Units.RPM));
        return Units.RPM.of(targetVelocityRpm);
      }
    };
  }
}
