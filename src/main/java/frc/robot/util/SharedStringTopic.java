package frc.robot.util;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import frc.robot.Robot;
import lombok.Getter;

/**
 * Wraps a pair of NetworkTables string topics for request/state communication.
 * Subscribes to
 * {@code topicBase/Request} and publishes to {@code topicBase/State} on the
 * dashboard NetworkTables
 * instance.
 */
public class SharedStringTopic {
  @Getter
  private final StringSubscriber subscriber;
  private final StringPublisher publisher;

  private static final String kRequestSuffix = "/Request";
  private static final String kStateSuffix = "/State";

  /**
   * Creates a shared string topic using the given base name.
   *
   * @param topicBase base name for the topic pair; "/Request" and "/State" are
   *                  appended for the
   *                  subscriber and publisher respectively
   */
  public SharedStringTopic(String topicBase) {
    this.subscriber = Robot.getDashboard().getStringTopic(topicBase + kRequestSuffix).subscribe("");
    this.publisher = Robot.getDashboard().getStringTopic(topicBase + kStateSuffix).publish();
  }

  /**
   * Returns the latest value from the Request topic.
   *
   * @return the current request string, or empty string if none has been
   *         published
   */
  public String getState() {
    return subscriber.get();
  }

  /**
   * Publishes the given value to the State topic.
   *
   * @param state the string to publish
   */
  public void setState(String state) {
    publisher.set(state);
  }
}
