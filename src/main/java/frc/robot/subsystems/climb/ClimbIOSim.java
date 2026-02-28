// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics simulation implementation of {@link ClimbIO}.
 *
 * <p>This class uses {@link DCMotorSim} to model the lift motor based on physical constants defined
 * in {@link ClimbConstants}. It updates the simulation state on every {@link
 * #updateInputs(ClimbIOInputs)} call and emulates limit switch behavior based on the simulated
 * position.
 */
public class ClimbIOSim implements ClimbIO {
  // Simulates the physical mechanics of the lift
  private final DCMotorSim liftSim;

  private Voltage appliedVoltage = Volts.of(0.0);

  /** Creates a new ClimbIOSim and initializes the motor simulation model. */
  public ClimbIOSim() {
    liftSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(liftGearbox, 0.02, 1.0), liftGearbox);
  }

  /**
   * Updates the simulation state and updates loggable inputs.
   *
   * @param inputs The inputs object to update with simulated data.
   */
  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // Apply voltage to simulation and clamp to 12V
    liftSim.setInputVoltage(MathUtil.clamp(appliedVoltage.in(Volts), -12.0, 12.0));
    // Advance simulation by 20ms (standard robot loop time)
    liftSim.update(0.02);

    // Update loggable inputs with simulated data
    inputs.connected = true;
    inputs.appliedVoltage = appliedVoltage;
    inputs.appliedCurrent = Amps.of(Math.abs(liftSim.getCurrentDrawAmps()));

    // Emulate limit switches based on simulated position
    // NOTE: Current simulation uses absolute value for positioning, needs better mapping
    inputs.lowerLimit =
        appliedVoltage.lt(Volts.of(0.0)) && Math.abs(liftSim.getAngularPositionRad()) > 10.0;
    inputs.upperLimit =
        appliedVoltage.gt(Volts.of(0.0)) && Math.abs(liftSim.getAngularPositionRad()) > 10.0;
  }

  @Override
  public void setLiftVoltage(Voltage voltage) {
    appliedVoltage = voltage;
  }
}
