// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simulation implementation of {@link HoodIO}.
 *
 * <p>This class simulates the movement of the hood actuator over time based on its configured
 * maximum velocity, allowing for realistic testing of shooter command logic without physical
 * hardware.
 */
public class HoodIOSim implements HoodIO {
  private Distance currentLength = servoMinLength;
  private Distance targetLength = servoMinLength;
  private double lastTimestamp = 0.0;

  /**
   * Updates inputs by modeling the movement of the hood actuator in the simulator.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double currentTime = Timer.getFPGATimestamp();
    double dt = lastTimestamp == 0 ? 0.02 : currentTime - lastTimestamp;
    lastTimestamp = currentTime;

    // Kinematic Model: Move currentLength toward targetLength based on max velocity
    double errorMeters = targetLength.minus(currentLength).in(Meters);
    double maxStepMeters = maxServoVelocity.in(MetersPerSecond) * dt;

    // Calculate distance to move this frame
    double moveMeters = MathUtil.clamp(errorMeters, -maxStepMeters, maxStepMeters);
    currentLength = currentLength.plus(Meters.of(moveMeters));

    // Update logged inputs
    inputs.currentLength = currentLength;
    inputs.targetLength = targetLength;

    // Determine if the mechanism has reached the target length
    inputs.atSetpoint = Math.abs(currentLength.minus(targetLength).in(Meters)) < 0.0005;
  }

  /**
   * Sets the target length for the simulated hood actuator.
   *
   * @param length The target length for the actuator.
   */
  @Override
  public void setLength(Distance length) {
    // Clamp requested length within physical limits
    this.targetLength =
        Meters.of(
            MathUtil.clamp(
                length.in(Meters), servoMinLength.in(Meters), servoMaxLength.in(Meters)));
  }
}
