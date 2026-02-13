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

public class HoodIOSim implements HoodIO {
  private Distance currentLength = servoMinLength;
  private Distance targetLength = servoMinLength;
  private double lastTimestamp = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double currentTime = Timer.getFPGATimestamp();
    double dt = lastTimestamp == 0 ? 0.02 : currentTime - lastTimestamp;
    lastTimestamp = currentTime;

    double errorMeters = targetLength.minus(currentLength).in(Meters);
    double maxStepMeters = maxServoVelocity.in(MetersPerSecond) * dt;
    double moveMeters = MathUtil.clamp(errorMeters, -maxStepMeters, maxStepMeters);
    currentLength = currentLength.plus(Meters.of(moveMeters));

    inputs.currentLength = currentLength;
    inputs.targetLength = targetLength;

    inputs.atSetpoint = Math.abs(currentLength.minus(targetLength).in(Meters)) < 0.0005;
  }

  @Override
  public void setLength(Distance length) {
    this.targetLength =
        Meters.of(
            MathUtil.clamp(
                length.in(Meters), servoMinLength.in(Meters), servoMaxLength.in(Meters)));
  }
}
