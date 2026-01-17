package frc.robot.constant;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ShooterConstants {
	public static final int kShooterCurrentLimit = 30;
	public static final double kWheelRadius = 0.0508;

	public static final double kShooterP = 0.01;
	public static final double kShooterI = 0.0001;
	public static final double kShooterD = 0.0;
	public static final double kShooterIZ = 0.0;

	public static final boolean kShooterReversed = false;
	public static final double kShooterMotorRotationsPerRotation = 1.0;

	public static final int kShooterCanId = 0;
	public static final MotorType kShooterMotorType = MotorType.kBrushless;

	public static final LinearVelocity kShooterMaxVelocity = Units.MetersPerSecond.of(50.0);
	public static final LinearAcceleration kShooterMaxAcceleration = Units.MetersPerSecondPerSecond.of(100.0);

	public static final int kShooterOffByMs = 200;
}
