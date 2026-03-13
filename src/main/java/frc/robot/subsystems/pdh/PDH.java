// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.pdh;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FaultUtil;
import org.littletonrobotics.junction.Logger;

public class PDH extends SubsystemBase {
  private final PDHIO io;
  private final PDHIOInputsAutoLogged inputs = new PDHIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("PDH disconnected", AlertType.kError);
  private final Alert faultAlert = new Alert("PDH fault detected", AlertType.kError);

  public PDH(PDHIO io) {
    this.io = io;
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("PDH", inputs);
    disconnectedAlert.set(!inputs.connected);

    // Check for faults if connected
    if (inputs.connected) {
      if (!inputs.healthy) {
        faultAlert.setText(
            FaultUtil.getArrayString(
                "PDH Faults: ", FaultUtil.getPdhActiveFaults(inputs.status[0])));
      }
    } else {
      faultAlert.set(false);
    }
  }

  /**
   * Returns whether or not the subsystem is healthy
   *
   * @return True if the subsystem is healthy, false otherwise.
   */
  public boolean isHealthy() {
    return inputs.healthy && inputs.connected;
  }

  /** Clears all faults and warnings. */
  public void clearFaults() {
    io.clearFaults();
  }
}
