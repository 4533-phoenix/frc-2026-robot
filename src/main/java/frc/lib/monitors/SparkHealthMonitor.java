// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.monitors;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.util.FaultUtil;

/** Monitors the health of a Spark Max or Spark Flex motor controller. */
public class SparkHealthMonitor {
  private final String name;
  private final Alert disconnected;
  private final Alert fault;
  private final Alert warn;
  private final Alert stickyFault;
  private final Alert stickyWarn;

  private int lastFaults, lastStickyFaults, lastWarns, lastStickyWarns = 0;
  private final StringBuilder sb = new StringBuilder();

  /**
   * Creates a new SparkHealthMonitor.
   *
   * @param name The name of the motor for alert messages.
   */
  public SparkHealthMonitor(String name) {
    this.name = name;
    this.disconnected = new Alert(name + " motor disconnected", AlertType.kError);
    this.fault = new Alert(name + " motor active fault", AlertType.kError);
    this.warn = new Alert(name + " motor active warning", AlertType.kWarning);
    this.stickyFault = new Alert(name + " motor sticky fault", AlertType.kInfo);
    this.stickyWarn = new Alert(name + " motor sticky warning", AlertType.kInfo);
  }

  /**
   * Updates the health monitor with the latest motor status.
   *
   * @param isConnected Whether the motor is connected.
   * @param status An array containing[activeFaults, stickyFaults, activeWarnings, stickyWarnings].
   */
  public void update(boolean isConnected, int[] status) {
    disconnected.set(!isConnected);
    if (!isConnected) {
      fault.set(false);
      warn.set(false);
      return;
    }

    // Only generate strings if the integer bitfield has changed
    if (status[0] != lastFaults) {
      lastFaults = status[0];
      fault.set(lastFaults != 0);
      if (lastFaults != 0) {
        sb.setLength(0);
        sb.append(name).append(" Faults: ");
        FaultUtil.appendSparkFaults(sb, lastFaults);
        fault.setText(sb.toString());
      }
    }

    if (status[2] != lastWarns) {
      lastWarns = status[2];
      warn.set(lastWarns != 0);
      if (lastWarns != 0) {
        sb.setLength(0);
        sb.append(name).append(" Warnings: ");
        FaultUtil.appendSparkWarnings(sb, lastWarns);
        warn.setText(sb.toString());
      }
    }

    if (status[1] != lastStickyFaults) {
      lastStickyFaults = status[1];
      stickyFault.set(lastStickyFaults != 0);
      if (lastStickyFaults != 0) {
        sb.setLength(0);
        sb.append(name).append(" Sticky Faults: ");
        FaultUtil.appendSparkFaults(sb, lastStickyFaults);
        stickyFault.setText(sb.toString());
      }
    }

    if (status[3] != lastStickyWarns) {
      lastStickyWarns = status[3];
      stickyWarn.set(lastStickyWarns != 0);
      if (lastStickyWarns != 0) {
        sb.setLength(0);
        sb.append(name).append(" Sticky Warnings: ");
        FaultUtil.appendSparkWarnings(sb, lastStickyWarns);
        stickyWarn.setText(sb.toString());
      }
    }
  }
}
