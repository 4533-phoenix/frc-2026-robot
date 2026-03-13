// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.pdh;

import edu.wpi.first.math.filter.Debouncer;
import org.littletonrobotics.conduit.ConduitApi;

/** Real IO implementation for the PDH subsystem. */
public class PDHIOReal implements PDHIO {
  private final ConduitApi conduit = ConduitApi.getInstance();
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  @Override
  public void updateInputs(PDHIOInputs inputs) {
    inputs.connected = connectedDebounce.calculate(conduit.getPDPVoltage() > 0.0);
    inputs.status[0] = (int) conduit.getPDPFaults();
    inputs.healthy = inputs.status[0] == 0;
    inputs.status[1] = (int) conduit.getPDPStickyFaults();
  }

  @Override
  public void clearFaults() {
    // Find a way to clear these but ak hides the handle
  }
}
