// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Gyro extends SubsystemBase {
  private final GyroIO io;
  private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

  /**
   * Creates a new Vision subsystem.
   *
   * @param io The abstraction layer for the vision hardware (e.g., Limelight, PhotonVision).
   * @param drive The Drive subsystem instance for updating pose estimates.
   */
  public Gyro(GyroIO io) {
    this.io = io;
  }

  /**
   * Processes vision measurements, filters invalid data, updates the drive pose estimator, and
   * checks camera status.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gyro", inputs);
  }
}
