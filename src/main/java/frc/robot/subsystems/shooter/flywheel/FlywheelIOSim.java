// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of {@link FlywheelIO}.
 *
 * <p>This class simulates the physical behavior of a flywheel using WPILib's {@link FlywheelSim}.
 * It calculates the necessary voltage to achieve a target velocity using a closed-loop PID
 * controller combined with a feedforward model.
 */
public class FlywheelIOSim implements FlywheelIO {
  // Physics model based on gearbox type, moment of inertia, and gear reduction
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              flywheelGearbox, flywheelMOI.in(KilogramSquareMeters), flywheelReduction),
          flywheelGearbox,
          flywheelReduction);

  // Closed-loop controller for velocity precision
  private final PIDController pid = new PIDController(flywheelKp, flywheelKi, flywheelKd);
  // Feedforward model to predict required voltage based on desired speed
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(flywheelKs, flywheelKv);

  private Voltage appliedVoltage = Volts.of(0.0);
  private boolean closedLoop = false;
  private AngularVelocity velocitySetpoint = RadiansPerSecond.of(0.0);

  /**
   * Updates inputs by simulating the flywheel physics model and calculating control efforts.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      // Calculate voltage based on PID error and feedforward prediction
      double currentRps = sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
      double setpointRps = velocitySetpoint.in(RotationsPerSecond);

      appliedVoltage =
          Volts.of(
              MathUtil.clamp(
                  pid.calculate(currentRps, setpointRps) + ff.calculate(setpointRps), 
                  -12.0, 
                  12.0));
    }

    // Apply voltage to the physics simulation and update state over the time step (dt)
    sim.setInputVoltage(appliedVoltage.in(Volts));
    final double dt = 0.02; // 20ms simulation step
    sim.update(dt);

    // Populate logged inputs
    inputs.connected = true;
    inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
    inputs.appliedVoltage = appliedVoltage;
    inputs.appliedCurrent = Amps.of(sim.getCurrentDrawAmps());
  }

  /**
   * Enables closed-loop velocity control for the flywheel.
   *
   * @param velocity The target angular velocity.
   */
  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    closedLoop = true;
    velocitySetpoint = velocity;
    pid.reset(); // Reset PID state to avoid integral windup on change
  }

  /**
   * Stops the flywheel motor by setting voltage to zero and disabling closed-loop control.
   */
  @Override
  public void stop() {
    closedLoop = false;
    appliedVoltage = Volts.of(0.0);
  }
}
