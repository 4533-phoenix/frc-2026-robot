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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Hood", inputs);
  }

  private static Distance convertLaunchAngleToServoPosition(Angle launchAngle) {
    // 1. Convert Launch Angle (0-90) to the Plate's angle relative to horizon
    // If launch is 77.125, plate is 90.0 (vertical)
    Angle plateAngle = launchAngle.plus(crankTangentToLaunchAngle);

    // 2. Calculate the required internal triangle angle (Theta)
    // Because extending the servo pushes the plate DOWN, the relationship is
    // inverse:
    // As plateAngle decreases (towards horizon), internalTheta must increase.
    Angle internalTheta = mechanismTotalAngle.minus(plateAngle);

    // 3. Law of Cosines to solve for required servo length (Side c)
    double a = groundLinkDistance.in(Inches);
    double b = crankArmLength.in(Inches);
    double cosTheta = Math.cos(internalTheta.in(Radians));

    double servoLengthSquared = (a * a) + (b * b) - (2 * a * b * cosTheta);
    double servoLength = Math.sqrt(Math.max(0, servoLengthSquared));

    // 4. Clamp to valid range (100-200 duty cycle would be: 100 + (position * 100))
    return Inches.of(
        MathUtil.clamp(servoLength, servoMinLength.in(Inches), servoMaxLength.in(Inches)));
  }

  public void setLaunchAngle(Angle angle) {
    io.setLength(convertLaunchAngleToServoPosition(angle));
  }

  public void retract() {
    io.setLength(servoMinLength);
  }

  public boolean isAtSetpoint() {
    return inputs.atSetpoint;
  }
}
