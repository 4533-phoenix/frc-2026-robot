// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;

public class HoodIOServo implements HoodIO {
  private final Servo servo = new Servo(hoodServoChannel);
  private Distance currentLength = servoMinLength;
  private Distance targetLength = servoMinLength;
  private double lastTimestamp = 0.0;

  public HoodIOServo() {
    servo.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double dt = lastTimestamp == 0 ? 0.02 : currentTime - lastTimestamp;
    lastTimestamp = currentTime;

    // Logic: Move currentLength toward targetLength based on max velocity
    double maxStep = maxServoVelocity.in(MetersPerSecond) * dt;
    double delta = targetLength.minus(currentLength).in(Meters);

    double step = MathUtil.clamp(delta, -maxStep, maxStep);
    currentLength = currentLength.plus(Meters.of(step));

    inputs.currentLength = currentLength;
    inputs.targetLength = targetLength;
    inputs.atSetpoint = Math.abs(delta) < 0.001; // 1mm tolerance
  }

  @Override
  public void setLength(Distance length) {
    Distance clampedLength =
        Meters.of(
            MathUtil.clamp(
                length.in(Meters), servoMinLength.in(Meters), servoMaxLength.in(Meters)));
    this.targetLength = clampedLength;

    double min = servoMinLength.in(Inches);
    double max = servoMaxLength.in(Inches);
    double val = (clampedLength.in(Inches) - min) / (max - min);

    servo.set(MathUtil.clamp(val, 0.0, 1.0));
  }
}
