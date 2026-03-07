package frc.robot.constant;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class ClimberConstants {

  public static enum ClimberPosition {
    L1(new Distance[] { Units.Meters.of(0.0), Units.Meters.of(1.0) }),
    L2(new Distance[] { Units.Meters.of(1.0), Units.Meters.of(0.0) });

    public final Distance[] positionMoveSequence;

    ClimberPosition(Distance[] positionMoveSequence) {
      this.positionMoveSequence = positionMoveSequence;
    }
  }

  public static final int climberMotorID = 8;
  public static final MotorType climberMotorType = MotorType.kBrushless;
  public static final boolean climberMotorInverted = true;

  public static final double gearRatio = 10.0;
  public static final Distance gearDiameter = Units.Meters.of(0.0254);
}
