package frc.robot.util.lighting.internal;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.util.lighting.LedColor;
import frc.robot.util.lighting.LightsEngine.LedSegment;

import java.util.List;

public class CandleLightsTransport implements LightsTransport {
  private final CANdle candle;

  public CandleLightsTransport(CANdle candle) {
    this.candle = candle;
  }

  @Override
  public void writeSegments(List<LedSegment> segments) {
    for (LedSegment segment : segments) {
      LedColor color = segment.color();
      candle.setControl(
          new SolidColor(segment.startInclusive(), segment.endInclusive())
              .withColor(new RGBWColor(color.red(), color.green(), color.blue(), color.white())));
    }
  }
}
