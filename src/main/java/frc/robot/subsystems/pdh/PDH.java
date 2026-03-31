// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.pdh;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.monitor.Monitored;
import frc.lib.monitor.checkers.PDHMonitor;
import org.littletonrobotics.junction.Logger;

/** Subsystem for monitoring the Power Distribution Hub (PDH) health and status. */
public class PDH extends SubsystemBase implements Monitored {
  private final PDHIO io;
  private final PDHIOInputsAutoLogged inputs = new PDHIOInputsAutoLogged();
  private final PDHMonitor healthMonitor = new PDHMonitor();

  /**
   * Creates a new PDH subsystem.
   *
   * @param io The IO implementation to use.
   */
  public PDH(PDHIO io) {
    this.io = io;
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("PDH", inputs);
    healthMonitor.update(inputs.connected, inputs.status[0], inputs.status[1]);
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
