// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for the shooter flywheel subsystem input/output abstraction.
 *
 * <p>This interface allows for interchangeable flywheel hardware (e.g., Falcon 500, SparkMax) and
 * comprehensive simulation support by standardizing how motor velocity is set and how electrical
 * data is retrieved.
 */
public interface FlywheelIO {
  /** Contains all of the inputs received from the flywheel hardware. */
  @AutoLog
  public static class FlywheelIOInputs {
    /** Whether the motor controller is successfully connected. */
    public boolean connected = false;

    /** The current angular velocity of the flywheel. */
    public AngularVelocity velocity = RadiansPerSecond.zero();

    /** The voltage currently being applied to the motor. */
    public Voltage appliedVoltage = Volts.zero();

    /** The current being drawn by the motor. */
    public Current appliedCurrent = Amps.zero();

    /** Whether the motor is functioning correctly. */
    public boolean healthy = true;

    /** Any active faults reported by the motor controller. */
    public String[] faults = new String[] {};

    /** Any active warnings reported by the motor controller. */
    public String[] warnings = new String[] {};
  }

  /**
   * Updates the set of loggable inputs with the latest data from the flywheel hardware.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /**
   * Commands the flywheel to spin at a specific angular velocity.
   *
   * @param velocity The target angular velocity for the flywheel.
   */
  public default void setAngularVelocity(AngularVelocity velocity) {}

  /** Stops the flywheel motor. */
  public default void stop() {}
}
