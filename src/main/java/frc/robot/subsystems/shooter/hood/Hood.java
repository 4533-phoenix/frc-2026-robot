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

  /**
   * Converts the desired launch angle (relative to horizontal) to the servo length,
   * accounting for the ball reflecting off the hood.
   *
   * The hood plate angle is set to (desired_launch_angle + incoming_angle) / 2.
   *
   * @param desiredLaunchAngle The angle you want the ball to leave at (deg from horizontal)
   * @return Servo length in Inches
   */
  private static Distance convertLaunchAngleToServoLength(Angle desiredLaunchAngle) {
    // Calculate the required hood plate angle
    Angle plateAngle = Degrees.of(
      (desiredLaunchAngle.in(Degrees) + incomingBallAngle.in(Degrees)) / 2.0
    );

    // Add mechanical offset if needed
    Angle plateAngleWithOffset = plateAngle.plus(crankTangentToLaunchAngle);

    // Calculate the required internal triangle angle (Theta)
    Angle internalTheta = mechanismTotalAngle.minus(plateAngleWithOffset);

    // Law of Cosines to solve for required servo length (Side c)
    double a = groundLinkDistance.in(Inches);
    double b = crankArmLength.in(Inches);
    double cosTheta = Math.cos(internalTheta.in(Radians));

    double servoLengthSquared = (a * a) + (b * b) - (2 * a * b * cosTheta);
    double servoLength = Math.sqrt(Math.max(0, servoLengthSquared));

    // Clamp to valid range
    return Inches.of(
        MathUtil.clamp(servoLength, servoMinLength.in(Inches), servoMaxLength.in(Inches)));
  }

  public void setLaunchAngle(Angle angle) {
    io.setLength(convertLaunchAngleToServoLength(angle));
  }

  public void retract() {
    io.setLength(servoMinLength);
  }

  public boolean isAtSetpoint() {
    return inputs.atSetpoint;
  }
}
