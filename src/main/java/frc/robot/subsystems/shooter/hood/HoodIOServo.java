// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;

/**
 * Real IO implementation for the shooter hood using a PWM servo.
 *
 * <p>This implementation maps the desired hood length to a servo position, and models the servo's
 * movement in {@link #updateInputs(HoodIOInputs)} to provide realistic feedback for simulation and
 * logging.
 */
public class HoodIOServo implements HoodIO {
  private final Servo servo = new Servo(SERVO_CHANNEL);
  private Distance currentLength = SERVO_MIN_LENGTH;
  private Distance targetLength = SERVO_MIN_LENGTH;
  private double lastTimestamp = 0.0;

  /** Creates a new HoodIOServo and configures the servo PWM bounds. */
  public HoodIOServo() {
    // Configure PWM bounds for specific servo hardware
    servo.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
  }

  /**
   * Updates inputs by modeling the servo movement over time.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double dt = lastTimestamp == 0 ? 0.02 : currentTime - lastTimestamp;
    lastTimestamp = currentTime;

    // Kinematic Model: Move currentLength toward targetLength based on max velocity
    double maxStep = MAX_SERVO_VELOCITY.in(MetersPerSecond) * dt;
    double delta = targetLength.minus(currentLength).in(Meters);

    // Clamp the step size to the maximum velocity
    double step = MathUtil.clamp(delta, -maxStep, maxStep);
    currentLength = currentLength.plus(Meters.of(step));

    // Log the current physical state of the actuator
    inputs.currentLength = currentLength;
    inputs.targetLength = targetLength;
    inputs.atSetpoint = Math.abs(delta) < HOOD_LENGTH_TOLERANCE.in(Meters);
  }

  /**
   * Commands the servo to a specific position corresponding to a desired hood length.
   *
   * @param length The target length for the actuator.
   */
  @Override
  public void setLength(Distance length) {
    // Clamp requested length within physical limits
    Distance clampedLength =
        Meters.of(
            MathUtil.clamp(
                length.in(Meters), SERVO_MIN_LENGTH.in(Meters), SERVO_MAX_LENGTH.in(Meters)));
    this.targetLength = clampedLength;

    // Map the length to a 0.0 - 1.0 servo value
    double min = SERVO_MIN_LENGTH.in(Inches);
    double max = SERVO_MAX_LENGTH.in(Inches);
    double val = (clampedLength.in(Inches) - min) / (max - min);

    // Set the servo position
    servo.set(MathUtil.clamp(val, 0.0, 1.0));
  }
}
