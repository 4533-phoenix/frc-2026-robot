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

public class FlywheelIOSim implements FlywheelIO {
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              flywheelGearbox, flywheelMOI.in(KilogramSquareMeters), flywheelReduction),
          flywheelGearbox,
          flywheelReduction);
  private final PIDController pid = new PIDController(flywheelKp, flywheelKi, flywheelKd);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(flywheelKs, flywheelKv);

  private Voltage appliedVoltage = Volts.of(0.0);
  private boolean closedLoop = false;
  private AngularVelocity velocitySetpoint = RadiansPerSecond.of(0.0);

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      appliedVoltage =
          Volts.of(
              MathUtil.clamp(
                  pid.calculate(
                          sim.getAngularVelocityRadPerSec(), velocitySetpoint.in(RadiansPerSecond))
                      + ff.calculate(velocitySetpoint.in(RadiansPerSecond)),
                  -12.0,
                  12.0));
    }

    sim.setInputVoltage(appliedVoltage.in(Volts));
    final double dt = 0.02;
    sim.update(dt);

    inputs.connected = true;
    inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
    inputs.appliedVoltage = appliedVoltage;
    inputs.appliedCurrent = Amps.of(sim.getCurrentDrawAmps());
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    closedLoop = true;
    velocitySetpoint = velocity;
    pid.reset();
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVoltage = Volts.of(0.0);
  }
}
