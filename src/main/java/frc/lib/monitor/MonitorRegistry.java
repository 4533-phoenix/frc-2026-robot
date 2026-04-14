// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.monitor;

import java.util.ArrayList;
import java.util.List;

/** A centralized registry for all monitored components on the robot. */
public final class MonitorRegistry {
  private static final List<Monitored> monitors = new ArrayList<>();

  private MonitorRegistry() {}

  /**
   * Registers a monitorable component to be checked by the superstructure.
   *
   * @param monitor The component to register.
   */
  public static void register(Monitored monitor) {
    if (!monitors.contains(monitor)) {
      monitors.add(monitor);
    }
  }

  /**
   * Gets all registered monitors.
   *
   * @return The list of registered monitors.
   */
  public static List<Monitored> getMonitors() {
    return monitors;
  }

  /**
   * Returns whether or not all registered monitored components are healthy.
   *
   * @return True if all components are healthy, false otherwise.
   */
  public static boolean isHealthy() {
    for (Monitored monitor : monitors) {
      if (!monitor.isHealthy()) {
        return false;
      }
    }
    return true;
  }

  /** Clears all faults and warnings from all registered monitored components. */
  public static void clearFaults() {
    for (Monitored monitor : monitors) {
      monitor.clearFaults();
    }
  }
}
