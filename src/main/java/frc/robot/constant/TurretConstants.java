package frc.robot.constant;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class TurretConstants {
	public static final int kTurretCanId = 27;
	public static final int kTurretCurrentLimit = 30;
	public static final double feedForwardFactor = 1.0;
	public static final Angle kTurretTheta = Units.Degrees.of(45.0);
	public static final double kTurretMotorRotationsPerRotation = 16.0;
	public static final MotorType kTurretMotorType = MotorType.kBrushless;

	public static final double kTurretP = 1;
	public static final double kTurretI = 0.01;
	public static final double kTurretD = 0.0;
	public static final double kTurretIZ = 0.0;

	public static final AngularVelocity kTurretMaxVelocity = Units.RadiansPerSecond.of(4.0);
	public static final AngularAcceleration kTurretMaxAcceleration = Units.RadiansPerSecondPerSecond.of(4.0);

	public static final int kTurretOffByMs = 200;
	public static final Translation2d turretPositionInRobot = new Translation2d(0, 1);
}
