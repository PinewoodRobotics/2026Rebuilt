package frc.robot.util.lighting.internal;

import java.util.List;

import frc.robot.util.lighting.LightsEngine.LedSegment;

public interface LightsTransport {
  void writeSegments(List<LedSegment> segments);
}
