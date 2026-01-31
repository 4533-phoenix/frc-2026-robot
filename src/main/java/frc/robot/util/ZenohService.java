package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import io.zenoh.Config;
import io.zenoh.Session;
import io.zenoh.Zenoh; // Import the main Zenoh entry point
import io.zenoh.keyexpr.KeyExpr;
import io.zenoh.pubsub.Publisher;
import java.util.HashMap;
import java.util.Map;

public class ZenohService {
  private static ZenohService instance;
  private Session session;
  private final Map<String, Publisher> publishers = new HashMap<>();
  private volatile boolean publishErrorLogged = false;

  public static synchronized ZenohService getInstance() {
    if (instance == null) {
      instance = new ZenohService();
    }
    return instance;
  }

  private ZenohService() {
    if (Constants.currentMode == Constants.Mode.REPLAY) return;

    try {
      System.out.println("[Zenoh] Opening Session...");
      Config config = Config.loadDefault();
      config.insertJson5("listen/endpoints", "['tcp/0.0.0.0:7447']");
      this.session = Zenoh.open(config);
      System.out.println("[Zenoh] Session Active.");
    } catch (Throwable t) {
      System.err.println("[Zenoh] Failed to open session: " + t.getMessage());
      t.printStackTrace();
      this.session = null;
    }
  }

  public void publish(String path, String value) {
    if (session == null) return; // Check if session is actually valid

    try {
      Publisher pub = publishers.get(path);
      if (pub == null) {
        // declaredPublisher returns a Publisher directly in recent Java bindings
        // autocanonize is fine, but ensure 'path' is clean to save CPU
        pub = session.declarePublisher(KeyExpr.autocanonize(path));

        if (pub != null) {
          publishers.put(path, pub);
        } else {
          return;
        }
      }

      // FIX 2: Check validity before putting
      pub.put(value);
      publishErrorLogged = false;

    } catch (Exception e) {
      if (!publishErrorLogged) {
        System.err.println("[Zenoh] Telemetry publish error: " + e.getMessage());
        e.printStackTrace();
        publishErrorLogged = true;
      }
    }
  }

  public void updateTelemetry() {
    publish("frc/robot/battery", String.valueOf(RobotController.getBatteryVoltage()));
  }

  public void close() {
    // FIX 3: Proper cleanup to allow restart
    if (session != null) {
      try {
        session.close();
      } catch (Exception e) {
        System.err.println("[Zenoh] Error closing session: " + e.getMessage());
      }
      session = null;
    }
    publishers.clear(); // Clear stale publishers
    instance = null; // Reset singleton
  }
}
