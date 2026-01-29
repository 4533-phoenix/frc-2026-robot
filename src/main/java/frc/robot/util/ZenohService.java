package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import io.zenoh.Config;
import io.zenoh.Session;
import io.zenoh.exceptions.ZError;
import io.zenoh.keyexpr.KeyExpr;
import io.zenoh.pubsub.Publisher;
import java.util.HashMap;
import java.util.Map;

public class ZenohService {
  private static ZenohService instance;
  private Session session;
  private final Map<String, Publisher> publishers = new HashMap<>();

  public static ZenohService getInstance() {
    if (instance == null) instance = new ZenohService();
    return instance;
  }

  private ZenohService() {
    // Don't open network ports during Log Replay
    if (Constants.currentMode == Constants.Mode.REPLAY) return;

    try {
      System.out.println("[Zenoh] Opening Session...");
      this.session = Session.Companion.open$zenoh_java(Config.loadDefault());
      System.out.println("[Zenoh] Session Active.");
    } catch (Exception e) {
      System.err.println("[Zenoh] Failed to open session: " + e.getMessage());
    }
  }
  /** Publishes a string to a specific path */
  public void publish(String path, String value) {
    if (session == null) return;
    try {
      Publisher pub = publishers.get(path);
      if (pub == null) {
        try {
          pub = session.declarePublisher(KeyExpr.autocanonize(path));
          if (pub != null) {
            publishers.put(path, pub);
          } else {
            return;
          }
        } catch (ZError e) {
          System.err.println(
              "[Zenoh] Failed to declare publisher for " + path + ": " + e.getMessage());
          return;
        }
      }
      if (pub != null) {
        pub.put(value);
      }
    } catch (Exception e) {
      // Suppress repeating errors in periodic
    }
  }

  /** Helper to publish battery voltage as an example */
  public void updateTelemetry() {
    publish("frc/robot/battery", String.valueOf(RobotController.getBatteryVoltage()));
  }

  public void close() {
    if (session != null) session.close();
  }
}
