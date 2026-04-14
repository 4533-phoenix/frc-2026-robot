// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.pdh;

/** Simulated IO implementation for the PDH subsystem. */
public class PDHIOSim implements PDHIO {
  @Override
  public void updateInputs(PDHIOInputs inputs) {
    inputs.connected = true;
    inputs.healthy = true;
  }
}
