package frc.lib.monitors;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.hardware.GyroType;
import frc.lib.util.FaultUtil;
import frc.robot.subsystems.drive.gyro.GyroIO.GyroIOInputs;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Monitors the health of one or more gyroscopes using agnostic parallel arrays. Employs integer
 * gatekeeping to achieve zero memory allocation during standard operation.
 */
public class GyroHealthMonitor {
  private final List<Alert> activeAlerts = new ArrayList<>();
  private final List<Alert> stickyAlerts = new ArrayList<>();
  private final Alert totalFailureAlert = new Alert("Total Drive Gyro Failure", AlertType.kError);

  private int[] lastActive = new int[0];
  private int[] lastSticky = new int[0];
  private final StringBuilder sb = new StringBuilder();

  /**
   * Updates the health alerts based on current IO inputs.
   *
   * @param inputs The hardware-agnostic gyro inputs.
   */
  public void update(GyroIOInputs inputs) {
    // Sync Alert Pool and cache arrays if hardware configuration changes
    if (activeAlerts.size() != inputs.types.length) {
      syncPool(inputs.types.length);
    }

    // Iterate through reported sensors
    for (int i = 0; i < inputs.types.length; i++) {
      int activeBits = inputs.activeFaults[i];
      int stickyBits = inputs.stickyFaults[i];
      GyroType type = inputs.types[i];

      if (activeBits != lastActive[i]) {
        lastActive[i] = activeBits;
        Alert a = activeAlerts.get(i);

        // If bits are 0, hide the alert. Otherwise, update the text.
        a.set(activeBits != 0);
        if (activeBits != 0) {
          sb.setLength(0);
          sb.append("Gyro ").append(i).append(" [").append(type).append("] Active: ");
          FaultUtil.appendGyroFaults(sb, type, activeBits);
          a.setText(sb.toString());
        }
      }

      // Sticky Fault Processing
      if (stickyBits != lastSticky[i]) {
        lastSticky[i] = stickyBits;
        Alert s = stickyAlerts.get(i);

        s.set(stickyBits != 0);
        if (stickyBits != 0) {
          sb.setLength(0);
          sb.append("Gyro ").append(i).append(" [").append(type).append("] Sticky: ");
          FaultUtil.appendGyroFaults(sb, type, stickyBits);
          s.setText(sb.toString());
        }
      }
    }

    totalFailureAlert.set(!inputs.connected);
  }

  /**
   * Resizes the internal alert pool and bitfield caches. Only runs when the number of gyros
   * reported by the IO layer changes.
   */
  private void syncPool(int size) {
    activeAlerts.clear();
    stickyAlerts.clear();
    for (int i = 0; i < size; i++) {
      activeAlerts.add(new Alert("Gyro " + i + " Fault", AlertType.kWarning));
      stickyAlerts.add(new Alert("Gyro " + i + " Sticky Fault", AlertType.kInfo));
    }
    lastActive = new int[size];
    lastSticky = new int[size];
    Arrays.fill(lastActive, -1);
    Arrays.fill(lastSticky, -1);
  }
}
