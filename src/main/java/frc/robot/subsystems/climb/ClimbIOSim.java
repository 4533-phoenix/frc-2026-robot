// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO {
  private final DCMotorSim liftSim;

  private Voltage appliedVoltage = Volts.of(0.0);

  public ClimbIOSim() {
    liftSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(liftGearbox, 0.02, 1.0), liftGearbox);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    liftSim.setInputVoltage(MathUtil.clamp(appliedVoltage.in(Volts), -12.0, 12.0));
    liftSim.update(0.02);

    inputs.connected = true;
    inputs.appliedVoltage = appliedVoltage;
    inputs.appliedCurrent = Amps.of(Math.abs(liftSim.getCurrentDrawAmps()));

    inputs.lowerLimit =
        appliedVoltage.lt(Volts.of(0.0)) && Math.abs(liftSim.getAngularPositionRad()) > 10.0;
    inputs.upperLimit =
        appliedVoltage.gt(Volts.of(0.0)) && Math.abs(liftSim.getAngularPositionRad()) > 10.0;
  }

  @Override
  public void setLiftVoltage(Voltage voltage) {
    appliedVoltage = voltage;
  }
}
