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

public class PDHHealthMonitor {
  private final Alert activeAlert = new Alert("PDH active fault", AlertType.kError);
  private final Alert stickyAlert = new Alert("PDH sticky fault", AlertType.kInfo);

  private int lastActive, lastSticky = 0;
  private final StringBuilder sb = new StringBuilder();

  public void update(boolean connected, int active, int sticky) {
    if (!connected) {
      activeAlert.set(false);
      return;
    }

    if (active != lastActive) {
      lastActive = active;
      activeAlert.set(active != 0);
      if (active != 0) {
        sb.setLength(0);
        sb.append("PDH Active: ");
        FaultUtil.appendPdhFaults(sb, active, false);
        activeAlert.setText(sb.toString());
      }
    }

    if (sticky != lastSticky) {
      lastSticky = sticky;
      stickyAlert.set(sticky != 0);
      if (sticky != 0) {
        sb.setLength(0);
        sb.append("PDH Sticky: ");
        FaultUtil.appendPdhFaults(sb, sticky, true);
        stickyAlert.setText(sb.toString());
      }
    }
  }
}