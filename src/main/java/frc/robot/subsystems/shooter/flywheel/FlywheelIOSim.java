// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(flywheelGearbox, flywheelMOI, flywheelReduction),
          flywheelGearbox,
          flywheelReduction);
  private final PIDController pid = new PIDController(flywheelKp, flywheelKi, flywheelKd);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(flywheelKs, flywheelKv);

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double velocitySetpoint = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          pid.calculate(sim.getAngularVelocityRadPerSec(), velocitySetpoint)
              + ff.calculate(velocitySetpoint);
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.connected = true;
    inputs.positionRad += sim.getAngularVelocityRadPerSec() * 0.02;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    closedLoop = true;
    velocitySetpoint = velocityRadPerSec;
    pid.reset();
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }
}
