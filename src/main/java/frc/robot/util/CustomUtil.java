package frc.robot.util;

import com.google.protobuf.Message;

public class CustomUtil {
  /**
   * Deserialize a protobuf message silently. If the deserialization fails, return
   * null.
   * 
   * @param bytes The bytes to deserialize.
   * @param clazz The class of the message to deserialize.
   * @return The deserialized message or null if the deserialization fails.
   */
  public static <T extends Message> T DeserializeSilent(byte[] bytes, Class<T> clazz) {
    try {
      var parseFromMethod = clazz.getMethod("parseFrom", byte[].class);
      @SuppressWarnings("unchecked")
      T message = (T) parseFromMethod.invoke(null, bytes);
      return message;
    } catch (Exception e) {
      e.printStackTrace();
      return null;
    }
  }
}
