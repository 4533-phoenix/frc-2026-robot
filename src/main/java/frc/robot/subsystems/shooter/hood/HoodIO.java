// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the shooter hood subsystem input/output abstraction. */
public interface HoodIO {
  /** Contains all of the inputs received from the hood hardware. */
  @AutoLog
  public static class HoodIOInputs {
    /** The current physical length of the hood actuator. */
    public Distance currentLength = Inches.of(0);

    /** The target length the hood actuator is moving towards. */
    public Distance targetLength = Inches.of(0);

    /** Whether the hood has reached its target setpoint. */
    public boolean atSetpoint = false;
  }

  /**
   * Updates the set of loggable inputs with the latest data from the hood hardware.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(HoodIOInputs inputs) {}

  /**
   * Commands the hood actuator to move to a specific length.
   *
   * @param length The target length for the actuator.
   */
  public default void setLength(Distance length) {}

  /** Retracts the hood mechanism to its minimum length (default state). */
  public default void retract() {
    setLength(SERVO_MIN_LENGTH);
  }
}
