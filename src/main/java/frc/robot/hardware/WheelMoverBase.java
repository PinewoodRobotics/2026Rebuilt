package frc.robot.hardware;

import org.pwrup.motor.WheelMover;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

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
  public abstract SwerveModuleState getState();

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
