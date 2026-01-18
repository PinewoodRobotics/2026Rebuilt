package frc.robot.constant;

import com.google.protobuf.InvalidProtocolBufferException;

import autobahn.client.NamedCallback;
import frc.robot.Robot;
import proto.status.Frontend.PIDFFUpdateMessage;

public class AutoUpdatingConstants {
    public static double kGeneralP = 0.0;
    public static double kGeneralI = 0.0;
    public static double kGeneralD = 0.0;
    public static double kGeneralFF = 0.0;

    static {
        final String kPIDFFUpdateTopic = "PIDFFUpdate";

        Robot.getCommunicationClient().subscribe(kPIDFFUpdateTopic, NamedCallback.FromConsumer(message -> {
            PIDFFUpdateMessage update;
            try {
                update = PIDFFUpdateMessage.parseFrom(message);
            } catch (InvalidProtocolBufferException e) {
                e.printStackTrace();
                return;
            }

            kGeneralP = update.getP();
            kGeneralI = update.getI();
            kGeneralD = update.getD();
            kGeneralFF = update.getFf();
        }));
    }
}
