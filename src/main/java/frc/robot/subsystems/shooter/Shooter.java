// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling the robot's shooter mechanism.
 *
 * <p>Responsible for controlling the angular velocity of the flywheels and the position of the
 * adjustable hood to regulate launch angle and distance.
 */
public class Shooter extends SubsystemBase {
  private final FlywheelIO flywheelIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private AngularVelocity targetVelocity = RadiansPerSecond.of(0.0);
  private boolean isShooting = false;

  private final Alert flywheelDisconnectedAlert =
      new Alert("Flywheel IO disconnected", AlertType.kWarning);

  /**
   * Creates a new Shooter subsystem.
   *
   * @param flywheelIO The abstraction layer for the flywheel hardware.
   * @param hoodIO The abstraction layer for the hood hardware.
   */
  public Shooter(FlywheelIO flywheelIO, HoodIO hoodIO) {
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    flywheelDisconnectedAlert.set(!flywheelInputs.connected);

    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
  }

  /**
   * Commands the shooter mechanisms to match a calculated state.
   *
   * @param state The desired {@link ShooterState} containing target flywheel speed and hood angle.
   */
  public void setTargetState(ShooterState state) {
    targetVelocity = state.flywheelSpeed();
    flywheelIO.setAngularVelocity(state.flywheelSpeed());
    // Convert angle to servo length for the physical mechanism
    hoodIO.setLength(convertHoodAngleToServoLength(state.hoodAngle()));
    isShooting = true;
  }

  /**
   * Converts the desired Hood Angle to Servo Length based on the physical mechanism.
   * Uses the Law of Cosines to determine the required servo length.
   *
   * @param hoodAngle The desired launch angle.
   * @return The required length for the hood actuator.
   */
  private static Distance convertHoodAngleToServoLength(Angle hoodAngle) {
    // Kinematic calculation for the hood mechanism
    Angle plateAngle = hoodAngle.plus(crankTangentToLaunchAngle);
    Angle internalTheta = mechanismTotalAngle.minus(plateAngle);

    double a = groundLinkDistance.in(Inches);
    double b = crankArmLength.in(Inches);
    double cosTheta = Math.cos(internalTheta.in(Radians));

    // Law of cosines: c^2 = a^2 + b^2 - 2ab*cos(C)
    double servoLengthSquared = (a * a) + (b * b) - (2 * a * b * cosTheta);
    double servoLength = Math.sqrt(Math.max(0, servoLengthSquared));

    return Inches.of(
        MathUtil.clamp(servoLength, servoMinLength.in(Inches), servoMaxLength.in(Inches)));
  }

  /**
   * Checks if the shooter is ready to launch a game piece.
   *
   * @return True if the flywheels are spun up, the hood is in position, and the shooter is active.
   */
  public boolean isReadyToShoot() {
    // Check if flywheel is within tolerance of target velocity
    double errorRps =
        targetVelocity.in(RadiansPerSecond) - flywheelInputs.velocity.in(RadiansPerSecond);
    boolean flywheelReady = Math.abs(errorRps) <= flywheelAngularTolerance.in(RadiansPerSecond);
    
    // Check if hood actuator has reached setpoint
    boolean hoodReady = hoodInputs.atSetpoint;

    return flywheelReady && hoodReady && isShooting;
  }

  /** Safely stops the flywheels and retracts the hood. */
  public void stop() {
    targetVelocity = RadiansPerSecond.of(0.0);
    flywheelIO.stop();
    hoodIO.retract();
    isShooting = false;
  }
}
