// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Real IO implementation for the shooter flywheel using a TalonFX motor controller (Falcon 500).
 *
 * <p>This implementation configures the motor for velocity closed-loop control and optimizes
 * CAN bus utilization by setting specific update frequencies for status signals.
 */
public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX talon = new TalonFX(flywheelMotorId);

  // Status signals for retrieving data from the TalonFX
  private final StatusSignal<Angle> position = talon.getPosition();
  private final StatusSignal<AngularVelocity> velocity = talon.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = talon.getMotorVoltage();
  private final StatusSignal<Current> current = talon.getStatorCurrent();

  /**
   * Creates a new FlywheelIOTalonFX and configures the motor controller.
   */
  public FlywheelIOTalonFX() {
    var config = new TalonFXConfiguration();
    // Configure motor direction and neutral behavior (Coast for flywheels)
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    // Set current limits to protect the motor
    config.CurrentLimits.StatorCurrentLimit = flywheelMotorCurrentLimit.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Configure PID gains for closed-loop velocity control
    config.Slot0.kP = flywheelKp;
    config.Slot0.kI = flywheelKi;
    config.Slot0.kD = flywheelKd;
    config.Slot0.kS = flywheelKs;
    config.Slot0.kV = flywheelKv;
    config.Slot0.kA = flywheelKa;
    talon.getConfigurator().apply(config);

    // Optimize CAN bus traffic by setting specific update rates (50Hz)
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);

    // Optimize bus utilization based on configured signals
    talon.optimizeBusUtilization();
  }

  /**
   * Updates inputs by refreshing status signals from the TalonFX.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Refresh signals from the CAN bus
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    // Check if all signals are valid and update inputs object
    inputs.connected = BaseStatusSignal.isAllGood(position, velocity, appliedVolts, current);
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = appliedVolts.getValue();
    inputs.appliedCurrent = current.getValue();
  }

  /**
   * Commands the TalonFX to spin at a specific angular velocity using closed-loop control.
   *
   * @param velocity The target angular velocity.
   */
  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    // Use VelocityVoltage control mode for precision
    talon.setControl(new VelocityVoltage(velocity.in(RotationsPerSecond)));
  }

  /**
   * Stops the flywheel motor by setting voltage output to zero.
   */
  @Override
  public void stop() {
    talon.setControl(new VoltageOut(0));
  }
}
