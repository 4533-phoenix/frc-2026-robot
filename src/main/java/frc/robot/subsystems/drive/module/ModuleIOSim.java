// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
  private PIDController turnController = new PIDController(turnSimP, 0, turnSimD);
  private Voltage driveFFVolts = Volts.of(0.0);
  private Voltage driveAppliedVoltage = Volts.of(0.0);
  private Voltage turnAppliedVoltage = Volts.of(0.0);

  public ModuleIOSim() {
    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
            driveGearbox);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, turnMotorReduction),
            turnGearbox);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVoltage =
          driveFFVolts.plus(
              Volts.of(driveController.calculate(driveSim.getAngularVelocityRadPerSec())));
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVoltage = Volts.of(turnController.calculate(turnSim.getAngularPositionRad()));
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVoltage.in(Volts), -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVoltage.in(Volts), -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePosition = driveSim.getAngularPosition();
    inputs.driveVelocity = driveSim.getAngularVelocity();
    inputs.driveAppliedVoltage = driveAppliedVoltage;
    inputs.driveCurrent = Amps.of(Math.abs(driveSim.getCurrentDrawAmps()));

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = turnSim.getAngularPosition();
    inputs.turnVelocity = turnSim.getAngularVelocity();
    inputs.turnAppliedVoltage = turnAppliedVoltage;
    inputs.turnCurrent = Amps.of(Math.abs(turnSim.getCurrentDrawAmps()));

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePosition.in(Radians)};
    inputs.odometryTurnPositions =
        new Rotation2d[] {Rotation2d.fromRadians(inputs.turnPosition.in(Radians))};
  }

  @Override
  public void setDriveOpenLoop(Voltage output) {
    driveClosedLoop = false;
    driveAppliedVoltage = Volts.of(MathUtil.clamp(output.in(Volts), -12.0, 12.0));
  }

  @Override
  public void setTurnOpenLoop(Voltage output) {
    turnClosedLoop = false;
    turnAppliedVoltage = Volts.of(MathUtil.clamp(output.in(Volts), -12.0, 12.0));
  }

  @Override
  public void setDriveVelocity(AngularVelocity velocity) {
    driveClosedLoop = true;
    driveFFVolts =
        Volts.of(
            driveSimKs * Math.signum(velocity.in(RadiansPerSecond))
                + driveSimKv * velocity.in(RadiansPerSecond));
    driveController.setSetpoint(velocity.in(RadiansPerSecond));
  }

  @Override
  public void setTurnPosition(Angle rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.in(Radians));
  }
}
