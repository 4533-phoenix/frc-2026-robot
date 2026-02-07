// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

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

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX talon = new TalonFX(flywheelMotorId);

  private final StatusSignal<Angle> position = talon.getPosition();
  private final StatusSignal<AngularVelocity> velocity = talon.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = talon.getMotorVoltage();
  private final StatusSignal<Current> current = talon.getStatorCurrent();

  public FlywheelIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = flywheelMotorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Slot0.kP = flywheelKp;
    config.Slot0.kI = flywheelKi;
    config.Slot0.kD = flywheelKd;
    config.Slot0.kS = flywheelKs;
    config.Slot0.kV = flywheelKv;
    talon.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);

    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    inputs.connected = BaseStatusSignal.isAllGood(position, velocity, appliedVolts, current);
    inputs.positionRad = position.getValue().in(edu.wpi.first.units.Units.Radians);
    inputs.velocityRadPerSec = velocity.getValue().in(edu.wpi.first.units.Units.RadiansPerSecond);
    inputs.appliedVolts = appliedVolts.getValue().in(edu.wpi.first.units.Units.Volts);
    inputs.currentAmps = current.getValue().in(edu.wpi.first.units.Units.Amps);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    talon.setControl(new VelocityVoltage(velocityRadPerSec / (2.0 * Math.PI)));
  }

  @Override
  public void stop() {
    talon.setControl(new VoltageOut(0));
  }
}
