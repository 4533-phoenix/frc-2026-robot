// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** IO interface for the climb hardware. */
public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    // Lift mechanism
    public boolean connected = false;
    public Voltage appliedVoltage = Volts.of(0.0);
    public Current appliedCurrent = Amps.of(0.0);

    // Limit switches
    public boolean lowerLimit = false;
    public boolean upperLimit = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Run the lift motors at the specified voltage. */
  public default void setLiftVoltage(Voltage voltage) {}
}
