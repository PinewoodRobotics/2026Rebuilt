package frc.robot.hardware;

import org.pwrup.motor.WheelMover;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

/**
 * Common base for swerve modules in this project.
 *
 * We extend {@link WheelMover} because PWRUP's {@code SwerveDrive} / {@code Wheel}
 * expects that type.
 */
public abstract class WheelMoverBase extends WheelMover {
  /** Module azimuth as a {@link Rotation2d}. */
  public abstract Rotation2d getRotation2d();

  /** Module distance + azimuth. */
  public abstract SwerveModulePosition getPosition();

  /** Module speed (m/s) + azimuth. */
  public abstract SwerveModuleVelocity getState();

  /** Resets encoders to zero (or equivalent). */
  public abstract void reset();

  /** Sets module drive speed using WPILib units (meters per second). */
  protected abstract void setSpeed(LinearVelocity mpsSpeed);

  /** Sets module azimuth using WPILib units (radians). */
  protected abstract void turnWheel(Angle newRotationRad);

  public abstract Angle getAngle();

  public abstract LinearVelocity getSpeed();

  public abstract Distance getDistance();

  /**
   * Default PWRUP API entrypoint: accepts raw doubles (radians, m/s), converts to
   * WPILib units, and delegates to the unit-typed helpers for clarity.
   */
  @Override
  public void drive(double angle, double speed) {
    drive(Units.Radians.of(angle), Units.MetersPerSecond.of(speed));
  }

  public void drive(Angle angle, LinearVelocity speed) {
    setSpeed(speed);
    turnWheel(angle);
  }
}
