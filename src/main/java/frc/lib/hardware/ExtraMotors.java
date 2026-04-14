// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;

/** Utility class for defining custom motor models not included in WPILib. */
public final class ExtraMotors {
  private ExtraMotors() {} // Prevent instantiation

  /**
   * Creates a {@link DCMotor} model for an AndyMark Snow Blower motor.
   *
   * @param numMotors The number of motors in the gearbox.
   * @return A modeled DCMotor instance based on datasheet specifications.
   */
  public static DCMotor getSnowBlower(int numMotors) {
    return new DCMotor(
        12.0, // Nominal Voltage (V)
        7.91, // Stall Torque (N-m)
        24.0, // Stall Current (A)
        5.0, // Free Current (A)
        RPM.of(100).in(RadiansPerSecond), // Free Speed (rad/s)
        numMotors);
  }
}
