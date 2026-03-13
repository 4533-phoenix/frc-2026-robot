// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.pdh;

import static frc.robot.subsystems.pdh.PDHConstants.*;

import edu.wpi.first.hal.PowerDistributionJNI;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/** Real IO implementation for the PDH subsystem. */
public class PDHIORev implements PDHIO {
  PowerDistribution pdh = new PowerDistribution(CAN_ID, ModuleType.kRev);

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  @Override
  public void updateInputs(PDHIOInputs inputs) {
    inputs.connected = connectedDebounce.calculate(pdh.getVoltage() > 0.0);
    inputs.status[0] = PowerDistributionJNI.getFaultsNative(pdh.getModule());
    inputs.healthy = inputs.status[0] == 0;
    inputs.status[1] = PowerDistributionJNI.getStickyFaultsNative(pdh.getModule());
  }

  @Override
  public void clearFaults() {
    pdh.clearStickyFaults();
  }
}
