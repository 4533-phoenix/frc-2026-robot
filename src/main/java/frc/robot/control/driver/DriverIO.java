// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.control.driver;

import org.littletonrobotics.junction.AutoLog;

public interface DriverIO {
  @AutoLog
  public static class DriverIOInputs {
    public double vxMetersPerSecond = 0.0;
    public double vyMetersPerSecond = 0.0;
    public double omegaRadiansPerSecond = 0.0;
    public boolean wantsAim = false;
    public boolean wantsShoot = false;
    public boolean wantsReset = false;
  }

  /** Updates the inputs based on the active profile. */
  public default void updateInputs(DriverIOInputs inputs, DriverProfile profile) {
    if (profile == null) return;

    var speeds = profile.getDesiredSpeeds();
    inputs.vxMetersPerSecond = speeds.vxMetersPerSecond;
    inputs.vyMetersPerSecond = speeds.vyMetersPerSecond;
    inputs.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
    inputs.wantsAim = profile.wantsAim();
    inputs.wantsShoot = profile.wantsShoot();
    inputs.wantsReset = profile.wantsReset();
  }
}
