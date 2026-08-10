package frc.robot.command.scoring;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.ShooterConstants;
import frc.robot.constant.TurretConstants;
import frc.robot.subsystem.TurretSubsystem;

public class ManualAimCommand extends Command {
  private final TurretSubsystem turretSubsystem;
  private final DoubleSupplier joystickAxisSupplier;
  private final double deadband;

  public ManualAimCommand(DoubleSupplier joystickAxisSupplier) {
    this(TurretSubsystem.GetInstance(), joystickAxisSupplier);
  }

  public ManualAimCommand(TurretSubsystem turretSubsystem, DoubleSupplier joystickAxisSupplier) {
    this(turretSubsystem, joystickAxisSupplier, 0.05);
  }

  public ManualAimCommand(
      TurretSubsystem turretSubsystem,
      DoubleSupplier joystickAxisSupplier,
      double deadband) {
    this.turretSubsystem = turretSubsystem;
    this.joystickAxisSupplier = joystickAxisSupplier;
    this.deadband = deadband;
    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    double rawAxis = joystickAxisSupplier.getAsDouble();
    double axis = MathUtil.applyDeadband(rawAxis, deadband);
    double clampedAxis = MathUtil.clamp(axis, -1.0, 1.0);

    double minRotations = TurretConstants.kTurretMinAngle.in(Units.Rotations);
    double maxRotations = TurretConstants.kTurretMaxAngle.in(Units.Rotations);
    double targetRotations = MathUtil.interpolate(minRotations, maxRotations, (-clampedAxis + 1.0) / 2.0);

    turretSubsystem.setTurretPosition(
        Units.Rotations.of(targetRotations),
        Units.Volts.of(0.0));

    Logger.recordOutput("Turret/ManualAimCommand/TargetRotations", targetRotations);
    Logger.recordOutput("Turret/ManualAimCommand/TargetRaw", rawAxis);
  }

  @Override
  public void initialize() {
    // No state needed. Setpoint is directly recomputed from joystick every loop.
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static double ReverseDirection(double speed) {
    if (speed > 0) {
      return 1 - speed;
    } else {
      return -1 + Math.abs(speed);
    }
  }
}
