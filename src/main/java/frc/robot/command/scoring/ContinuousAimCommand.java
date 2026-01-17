package frc.robot.command.scoring;

import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant.TurretConstants;
import frc.robot.subsystem.GlobalPosition;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.LocalMath;

public class ContinuousAimCommand extends Command {
  private final TurretSubsystem turretSubsystem;
  private final Supplier<Translation3d> targetGlobalPoseSupplier;
  private final Supplier<Pose2d> selfGlobalPoseSupplier;
  private final Supplier<Translation2d> currentRobotVelocitySupplier;
  private final Supplier<Translation2d> currentRobotAccelerationSupplier;
  private final Supplier<AngularVelocity> currentRobotYawVelocitySupplier;
  private final Supplier<AngularAcceleration> currentRobotYawAccelerationSupplier;

  public ContinuousAimCommand(Supplier<Translation3d> targetGlobalPoseSupplier,
      Supplier<Pose2d> selfGlobalPoseSupplier,
      Supplier<Translation2d> currentRobotVelocitySupplier,
      Supplier<Translation2d> currentRobotAccelerationSupplier,
      Supplier<AngularVelocity> currentRobotYawVelocitySupplier,
      Supplier<AngularAcceleration> currentRobotYawAccelerationSupplier) {
    this.turretSubsystem = TurretSubsystem.GetInstance();
    this.targetGlobalPoseSupplier = targetGlobalPoseSupplier;
    this.selfGlobalPoseSupplier = selfGlobalPoseSupplier;
    this.currentRobotVelocitySupplier = currentRobotVelocitySupplier;
    this.currentRobotAccelerationSupplier = currentRobotAccelerationSupplier;
    this.currentRobotYawVelocitySupplier = currentRobotYawVelocitySupplier;
    this.currentRobotYawAccelerationSupplier = currentRobotYawAccelerationSupplier;
    addRequirements(this.turretSubsystem);
  }

  public ContinuousAimCommand(Supplier<Translation3d> targetGlobalPoseSupplier) {
    this(targetGlobalPoseSupplier, GlobalPosition::Get, () -> new Translation2d(0, 0),
        () -> new Translation2d(0, 0),
        () -> AngularVelocity.ofRelativeUnits(0, Units.RadiansPerSecond),
        () -> AngularAcceleration.ofRelativeUnits(0, Units.RadiansPerSecondPerSecond));
  }

  @Override
  public void execute() {
    Pose2d selfPose = selfGlobalPoseSupplier.get();
    Translation3d targetGlobal = targetGlobalPoseSupplier.get();
    Translation2d selfTranslation = selfPose.getTranslation();
    Translation2d targetTranslation = targetGlobal.toTranslation2d();
    Translation2d target = LocalMath.fromGlobalToRelative(selfTranslation, targetTranslation);
    Translation2d velocity = currentRobotVelocitySupplier.get();

    double t_f = calculateTf(target.getNorm(), targetGlobal.getZ(), TurretConstants.kTurretTheta);

    // Predict where the target appears in the robot frame at impact time:
    // T_new = target - v_r * t_f - L
    // T_robot = R(w * t_f) * T_new
    // ------------------------------------------------------------
    Translation2d T_new = target
        .minus(velocity.times(t_f))
        .minus(TurretConstants.turretPositionInRobot);

    Matrix<N2, N2> R = fromAngularVelToMat(currentRobotYawVelocitySupplier.get(), t_f);
    Matrix<N2, N1> T_robot = R.times(T_new.toVector());
    Translation2d aimPoint = new Translation2d(T_robot.get(0, 0), T_robot.get(1, 0));
    // ------------------------------------------------------------

    Angle newAngle = Angle.ofRelativeUnits(aimPoint.getAngle().getRadians(), Units.Radians);

    AngularVelocity newAngleRate = calculateAimAngleRate(target.getNorm(), targetGlobal.getZ(),
        TurretConstants.kTurretTheta, velocity,
        target, TurretConstants.turretPositionInRobot, currentRobotAccelerationSupplier.get(),
        currentRobotYawVelocitySupplier.get(), currentRobotYawAccelerationSupplier.get());

    double feedForwardV = newAngleRate.in(Units.RadiansPerSecond) * TurretConstants.feedForwardFactor;

    turretSubsystem.setTurretPosition(newAngle, Voltage.ofRelativeUnits(feedForwardV, Units.Volts));

    logEverything(selfPose, targetGlobal, target, velocity, aimPoint, newAngle);
  }

  private Matrix<N2, N2> fromAngularVelToMat(AngularVelocity w, double time) {
    double delta = w.in(Units.RadiansPerSecond) * time;

    return new Matrix<N2, N2>(new SimpleMatrix(new double[][] {
        { Math.cos(delta), -Math.sin(delta) },
        { Math.sin(delta), Math.cos(delta) }
    }));
  }

  private void logEverything(
      Pose2d selfPose,
      Translation3d targetGlobal,
      Translation2d targetRelative,
      Translation2d robotVelocity,
      Translation2d aimPoint,
      Angle commandedAngle) {
    // Raw inputs
    Logger.recordOutput("Turret/AimCommand/SelfPose", selfPose);
    Logger.recordOutput("Turret/AimCommand/TargetGlobal", targetGlobal);
    Logger.recordOutput("Turret/AimCommand/RobotVelocity", robotVelocity);

    // Derived math
    Logger.recordOutput("Turret/AimCommand/TargetRelative", targetRelative);
    Logger.recordOutput("Turret/AimCommand/TargetDistanceMeters", targetRelative.getNorm());
    Logger.recordOutput("Turret/AimCommand/AimPoint", aimPoint);
    Logger.recordOutput("Turret/AimCommand/AimDistanceMeters", aimPoint.getNorm());

    // Commanded angle
    double goalRad = commandedAngle.in(Units.Radians);
    Logger.recordOutput("Turret/AimCommand/GoalAngleRad", goalRad);
    Logger.recordOutput("Turret/AimCommand/GoalAngleDeg", commandedAngle.in(Units.Degrees));

    // Turret feedback vs goal
    double currentRad = turretSubsystem.getTurretPosition().in(Units.Radians);
    double errorRad = Math.IEEEremainder(goalRad - currentRad, 2.0 * Math.PI); // wrap to [-pi, pi]
    Logger.recordOutput("Turret/AimCommand/CurrentAngleRad", currentRad);
    Logger.recordOutput("Turret/AimCommand/ErrorRad", errorRad);
    Logger.recordOutput("Turret/AimCommand/ErrorDeg", Math.toDegrees(errorRad));

    // Timing (kept for compatibility with existing dashboards)
    Logger.recordOutput("Turret/TimeLeftToReachPosition", turretSubsystem.getAimTimeLeftMs());
  }

  private double calculateTf(double X, double Y, Angle theta) {
    return Math.sqrt((X * Math.tan(theta.in(Units.Radians)) - Y) / 4.9);
  }

  private double sec(double x) {
    return 1 / Math.cos(x);
  }

  /**
   * Computes the <b>instantaneous angular rate</b> (time derivative) of the
   * <b>predicted impact-relative aim direction</b> to a target.
   *
   * <p>
   * This method exists to answer: <i>"If I fire right now, how fast is my
   * required aim angle changing right now?"</i> It differentiates the predicted
   * impact-relative 2D vector and then converts that vector derivative into an
   * angular derivative {@code d(alpha)/dt}.
   * </p>
   *
   * <h3>What was added / what this accounts for</h3>
   * <ul>
   * <li><b>Ballistic time-of-flight</b> {@code t_f} (simplified no-drag model)
   * and its derivative {@code d(t_f)/dt}.</li>
   * <li><b>Robot translation during flight</b>: subtracts {@code v_r * t_f} and
   * includes translational acceleration {@code a_r} when differentiating.</li>
   * <li><b>Robot yaw rotation during flight</b>: rotates by
   * {@code delta = w * t_f}
   * and includes both yaw-rate {@code w} and yaw angular acceleration {@code a}
   * in {@code d(delta)/dt}.</li>
   * <li><b>Turret/launcher offset</b> {@code L} from the robot reference
   * point.</li>
   * </ul>
   *
   * <p>
   * Conceptually this method differentiates, with respect to time at the instant
   * of firing, the
   * following predicted relative vector (expressed in the robot frame) at the
   * moment the projectile
   * reaches the target range:
   * </p>
   *
   * <pre>
   * T_new   = target - v_r * t_f - L
   * delta   = w * t_f
   * T_robot = R(delta) * T_new
   * alpha   = atan2(T_robot.y, T_robot.x)
   * return d(alpha)/dt
   * </pre>
   *
   * <p>
   * where {@code t_f} is the flight time derived from a simplified ballistic
   * model (no drag, constant
   * gravity) and {@code R(delta)} is the 2D rotation matrix for the yaw change
   * that occurs over the
   * flight time.
   * </p>
   *
   * <h3>Ballistic model</h3>
   * <p>
   * The flight time is computed from the vertical displacement equation:
   * </p>
   *
   * <pre>
   * Y = X * tan(theta) - (g / 2) * t_f ^ 2
   * </pre>
   *
   * <p>
   * using {@code g/2 = 4.9} (i.e., {@code g ≈ 9.8 m/s^2}) and solving for
   * {@code t_f}:
   * </p>
   *
   * <pre>
   * t_f = sqrt((X * tan(theta) - Y) / 4.9)
   * </pre>
   *
   * <p>
   * The method then computes {@code d_t_f} (the time-derivative of flight time)
   * via the chain rule,
   * using the current robot translational velocity components ({@code vx, vy})
   * and the derivative
   * of {@code tan(theta)} w.r.t. time (via {@code sec^2(theta)} terms) as encoded
   * in the expression.
   * </p>
   *
   * <h3>Frames, sign conventions, and units</h3>
   * <ul>
   * <li><b>{@code X}, {@code Y}</b>: scalar components used by the ballistic
   * model. In typical usage,
   * {@code X} is horizontal range to the target (meters) and {@code Y} is
   * vertical height difference
   * (meters, positive up). They must be consistent with {@code theta}.</li>
   * <li><b>{@code theta}</b>: launch/elevation angle (radians) used in the
   * ballistic equation.</li>
   * <li><b>{@code target}</b>: 2D target translation in the robot's reference
   * frame at the current
   * time (meters).</li>
   * <li><b>{@code L}</b>: 2D translation from robot reference point to
   * launcher/turret origin (meters).</li>
   * <li><b>{@code v_r}</b>: robot translational velocity in the same 2D frame as
   * {@code target} (m/s).</li>
   * <li><b>{@code a_r}</b>: robot translational acceleration in the same 2D frame
   * as {@code target} (m/s^2).</li>
   * <li><b>{@code w}</b>: robot yaw rate (rad/s). Positive direction must match
   * the rotation used
   * elsewhere in the codebase.</li>
   * <li><b>{@code a}</b>: robot yaw angular acceleration (rad/s^2).</li>
   * </ul>
   *
   * <h3>Return value</h3>
   * <p>
   * Returns {@code d(alpha)/dt}, the instantaneous time-derivative of the aim
   * angle {@code alpha}.
   * </p>
   *
   * <p>
   * <b>Important:</b> This method returns an {@link Angle} object for
   * convenience,
   * but the numeric value represents an <b>angular rate</b> (units of
   * radians/second), not a static angle. If you use this in feedforward, treat
   * {@code result.in(Units.Radians)} as {@code rad/s}.
   * </p>
   *
   * @param X
   *               Horizontal range used by the ballistic model (meters).
   * @param Y
   *               Vertical displacement used by the ballistic model (meters,
   *               positive up).
   * @param theta
   *               Launch/elevation angle used by the ballistic model (radians).
   * @param v_r
   *               Robot translational velocity in the same 2D frame as
   *               {@code target} (m/s).
   * @param target
   *               Target translation in the robot frame at the current instant
   *               (meters).
   * @param L
   *               Offset from robot reference point to launcher/turret origin
   *               (meters).
   * @param a_r
   *               Robot translational acceleration in the same 2D frame as
   *               {@code target} (m/s^2).
   * @param w
   *               Robot yaw angular velocity (rad/s).
   * @param a
   *               Robot yaw angular acceleration (rad/s^2).
   * @return An {@link Angle} whose numeric value (in radians) should be
   *         interpreted
   *         as {@code d(alpha)/dt} in radians/second.
   *
   *         <h3>Numerical / domain caveats</h3>
   *         <ul>
   *         <li><b>Sqrt domain</b>: requires
   *         {@code (X * tan(theta) - Y) / 4.9 >= 0}. If
   *         not, {@code t_f}
   *         becomes NaN.</li>
   *         <li><b>Sensitivity</b>: when {@code X * tan(theta) ≈ Y}, {@code t_f}
   *         approaches 0 and
   *         {@code d_t_f} can become very large (division by a small sqrt
   *         term).</li>
   *         <li><b>Model limitations</b>: ignores aerodynamic drag, spin, and
   *         changes in
   *         projectile speed;
   *         assumes robot velocities/accelerations are approximately constant
   *         over the
   *         flight.</li>
   *         </ul>
   */
  @SuppressWarnings("unused")
  private AngularVelocity calculateAimAngleRate(
      double X, double Y, Angle theta, Translation2d v_r, Translation2d target,
      Translation2d L, Translation2d a_r, AngularVelocity w, AngularAcceleration a) {
    double t_f = calculateTf(X, Y, theta);
    // double v_b = X / (Math.cos(theta.in(Units.Radians)) * t_f);

    double vx = v_r.getX();
    double vy = v_r.getY();

    double d_t_f = (1 / (2 * Math.sqrt((X * Math.tan(theta.in(Units.Radians)) - Y) / 4.9)))
        * (((vx * Math.tan(theta.in(Units.Radians)))
            + (X * Math.pow(sec(theta.in(Units.Radians)), 2)) - vy)
            / 4.9);

    Vector<N2> T_v = v_r.toVector().times(-1);
    Vector<N2> A_r = a_r.toVector();

    Vector<N2> T_new = target.toVector().minus(v_r.toVector().times(t_f)).minus(L.toVector());
    Vector<N2> d_T_new = T_v.minus(A_r).minus(v_r.toVector().times(d_t_f));

    double delta = w.in(Units.RadiansPerSecond) * t_f;
    double d_delta = a.in(Units.RadiansPerSecondPerSecond) * t_f + w.in(Units.RadiansPerSecond) * d_t_f;

    Matrix<N2, N2> R = new Matrix<N2, N2>(new SimpleMatrix(new double[][] {
        { Math.cos(delta), -Math.sin(delta) },
        { Math.sin(delta), Math.cos(delta) }
    }));

    Matrix<N2, N2> d_R = new Matrix<N2, N2>(new SimpleMatrix(new double[][] {
        { -Math.sin(delta) * d_delta, -Math.cos(delta) * d_delta },
        { Math.cos(delta) * d_delta, -Math.sin(delta) * d_delta }
    }));

    Matrix<N2, N1> T_new_R_mat = R.times(T_new);
    Vector<N2> T_new_R = new Vector<N2>(new SimpleMatrix(new double[][] {
        { T_new_R_mat.get(0, 0) },
        { T_new_R_mat.get(1, 0) }
    }));

    Matrix<N2, N1> d_T_new_r_mat = d_R.times(T_new).plus(R.times(d_T_new));

    Vector<N2> d_T_new_r = new Vector<N2>(new SimpleMatrix(new double[][] {
        { d_T_new_r_mat.get(0, 0) },
        { d_T_new_r_mat.get(1, 0) }
    }));

    double T_new_R_x = T_new_R.get(0, 0);
    double T_new_R_y = T_new_R.get(1, 0);

    double d_T_new_R_x = d_T_new_r.get(0, 0);
    double d_T_new_R_y = d_T_new_r.get(1, 0);

    double d_alpha_dt = (1 / (1 + Math.pow(T_new_R_y / T_new_R_x, 2)))
        * ((T_new_R_x * d_T_new_R_y - T_new_R_y * d_T_new_R_x) / Math.pow(T_new_R_x, 2));

    return AngularVelocity.ofRelativeUnits(d_alpha_dt, Units.RadiansPerSecond);
  }
}
