package frc.robot.constant;

import com.google.protobuf.InvalidProtocolBufferException;

import autobahn.client.NamedCallback;
import frc.robot.Robot;
import proto.status.Frontend.PIDFFUpdateMessage;

public class AutoUpdatingConstants {
    // These pid constants will be updated from the frontend on the topic of
    // "PIDFFUpdate"
    private static final String kPIDFFUpdateTopic = "PIDFFUpdate";

    public static double kGeneralP = 0.0;
    public static double kGeneralI = 0.0;
    public static double kGeneralD = 0.0;
    public static double kGeneralFF = 0.0;

    static {
        // actually update the constants here
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

    private static String a; // example for another auto updating constant
    static {
        // do the same exact thing as above but for the other auto updating constant
        // make sure to never do this to just one thing and bundle multiple constants
        // into one proto message instead.
    }
}
