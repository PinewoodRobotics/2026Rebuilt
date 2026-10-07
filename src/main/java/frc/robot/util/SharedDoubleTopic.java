package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.Robot;

/**
 * Double counterpart of {@link SharedStringTopic}: subscribes to
 * {@code topicBase/Request} and publishes to {@code topicBase/State} on the
 * dashboard NetworkTables instance.
 */
public class SharedDoubleTopic {
  private final DoubleSubscriber subscriber;
  private final DoublePublisher publisher;

  private static final String kRequestSuffix = "/Request";
  private static final String kStateSuffix = "/State";

  public SharedDoubleTopic(String topicBase, double requestDefault) {
    this.subscriber = Robot.getDashboard().getDoubleTopic(topicBase + kRequestSuffix).subscribe(requestDefault);
    this.publisher = Robot.getDashboard().getDoubleTopic(topicBase + kStateSuffix).publish();
  }

  public double getRequest() {
    return subscriber.get();
  }

  public void setState(double state) {
    publisher.set(state);
  }
}
