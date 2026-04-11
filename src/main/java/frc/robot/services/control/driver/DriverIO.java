// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.control.driver;

import org.littletonrobotics.junction.AutoLog;

/** Interface for driver input/output hardware abstraction. */
public interface DriverIO {
  /** Container for driver input values. */
  @AutoLog
  public static class DriverIOInputs {
    /** X velocity in meters per second. */
    public double vxMetersPerSecond = 0.0;

    /** Y velocity in meters per second. */
    public double vyMetersPerSecond = 0.0;

    /** Angular velocity in radians per second. */
    public double omegaRadiansPerSecond = 0.0;

    /** Whether the driver wants to aim. */
    public boolean wantsAim = false;

    /** Whether the driver wants to shoot. */
    public boolean wantsShoot = false;

    /** Whether the driver wants to reset the robot pose. */
    public boolean wantsReset = false;

    /** Whether the driver wants to unjam the indexer. */
    public boolean wantsUnjam = false;

    /** Whether the driver is connected. */
    public boolean connected = false;
  }

  /**
   * Updates the inputs based on the active profile.
   *
   * @param inputs The inputs object to populate.
   * @param profile The active driver profile.
   */
  public default void updateInputs(DriverIOInputs inputs, DriverProfile profile) {
    if (profile == null) return;

    var speeds = profile.getDesiredSpeeds();
    inputs.vxMetersPerSecond = speeds.vxMetersPerSecond;
    inputs.vyMetersPerSecond = speeds.vyMetersPerSecond;
    inputs.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
    inputs.wantsAim = profile.wantsAim();
    inputs.wantsShoot = profile.wantsShoot();
    inputs.wantsReset = profile.wantsReset();
    inputs.connected = profile.isConnected();
  }
}
