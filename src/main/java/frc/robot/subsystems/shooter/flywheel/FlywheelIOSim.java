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
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation implementation of {@link FlywheelIO}.
 *
 * <p>This class uses a real {@link TalonFX} device with its {@link TalonFXSimState} to simulate the
 * flywheel motor controller, backed by a {@link DCMotorSim} physics model. The TalonFX's built-in
 * closed-loop PID is used for velocity control, matching the real robot's {@link
 * FlywheelIOTalonFX}.
 */
public class FlywheelIOSim implements FlywheelIO {
  private final TalonFX talon = new TalonFX(flywheelMotorId);
  private final TalonFXSimState talonSim = talon.getSimState();

  // Status signals for retrieving data
  private final StatusSignal<Angle> position = talon.getPosition();
  private final StatusSignal<AngularVelocity> velocity = talon.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = talon.getMotorVoltage();
  private final StatusSignal<Current> current = talon.getStatorCurrent();

  // Physics model for the flywheel
  private final DCMotorSim physicsSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              flywheelGearbox, flywheelMOI.in(KilogramSquareMeters), flywheelReduction),
          flywheelGearbox);

  /** Creates a new FlywheelIOSim and configures the TalonFX (identical to FlywheelIOTalonFX). */
  public FlywheelIOSim() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.StatorCurrentLimit = flywheelMotorCurrentLimit.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kP = flywheelKp;
    config.Slot0.kI = flywheelKi;
    config.Slot0.kD = flywheelKd;
    config.Slot0.kS = flywheelKs;
    config.Slot0.kV = flywheelKv;
    config.Slot0.kA = flywheelKa;
    talon.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    talon.optimizeBusUtilization();
  }

  /**
   * Updates the physics simulation and populates inputs from the TalonFX sim state.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Set supply voltage for the sim
    talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Get the motor voltage output and feed it into the physics model
    physicsSim.setInputVoltage(talonSim.getMotorVoltageMeasure().in(Volts));
    physicsSim.update(0.02);

    // Feed physics results back into TalonFX sim state
    // DCMotorSim returns mechanism position/velocity (after gear ratio),
    // but TalonFX expects raw rotor position/velocity (before gear ratio)
    talonSim.setRawRotorPosition(physicsSim.getAngularPosition().times(flywheelReduction));
    talonSim.setRotorVelocity(physicsSim.getAngularVelocity().times(flywheelReduction));

    // Refresh status signals and populate inputs
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    inputs.connected = BaseStatusSignal.isAllGood(position, velocity, appliedVolts, current);
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = appliedVolts.getValue();
    inputs.appliedCurrent = current.getValue();
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    talon.setControl(new VelocityVoltage(velocity.in(RotationsPerSecond)));
  }

  @Override
  public void stop() {
    talon.setControl(new VoltageOut(0));
  }
}
