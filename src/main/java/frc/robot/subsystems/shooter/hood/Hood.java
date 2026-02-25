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

  private static Distance convertHoodAngleToServoLength(Angle hoodAngle) {
    // Convert Hood Angle (0-90) to the Plate's angle relative to horizon
    Angle plateAngle = hoodAngle.plus(crankTangentToLaunchAngle);

    // Calculate the required internal triangle angle
    Angle internalTheta = mechanismTotalAngle.minus(plateAngle);

    // Solve for required servo length
    double a = groundLinkDistance.in(Inches);
    double b = crankArmLength.in(Inches);
    double cosTheta = Math.cos(internalTheta.in(Radians));

    double servoLengthSquared = (a * a) + (b * b) - (2 * a * b * cosTheta);
    double servoLength = Math.sqrt(Math.max(0, servoLengthSquared));

    // Clamp to valid range
    return Inches.of(
        MathUtil.clamp(servoLength, servoMinLength.in(Inches), servoMaxLength.in(Inches)));
  }

  public void setHoodAngle(Angle angle) {
    io.setLength(convertHoodAngleToServoLength(angle));
  }

  public void retract() {
    io.setLength(servoMinLength);
  }

  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }
}
