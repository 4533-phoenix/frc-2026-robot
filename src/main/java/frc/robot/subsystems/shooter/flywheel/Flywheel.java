// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private AngularVelocity angularVelocitySetpoint = RadiansPerSecond.of(0.0);

  private final Alert disconnectedAlert = new Alert("Flywheel IO disconnected", AlertType.kWarning);

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);
    disconnectedAlert.set(!inputs.connected);
  }

  public void setAngularVelocity(AngularVelocity velocity) {
    angularVelocitySetpoint = velocity;
    io.setAngularVelocity(velocity);
  }

  public void setLinearVelocity(LinearVelocity velocity) {
    setAngularVelocity(
        RadiansPerSecond.of(velocity.in(MetersPerSecond) / flywheelWheelRadius.in(Meters)));
  }

  public AngularVelocity getAngularVelocity() {
    return inputs.velocity;
  }

  public LinearVelocity getLinearVelocity() {
    return MetersPerSecond.of(
        inputs.velocity.in(RadiansPerSecond) * flywheelWheelRadius.in(Meters));
  }

  public AngularVelocity getAngularVelocitySetpoint() {
    return angularVelocitySetpoint;
  }

  public boolean atSetpoint() {
    return angularVelocitySetpoint.minus(inputs.velocity).abs(RadiansPerSecond)
        <= flywheelAngularTolerance.in(RadiansPerSecond);
  }

  public void stop() {
    io.stop();
  }
}
