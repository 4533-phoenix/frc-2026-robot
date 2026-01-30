// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO {
  private final DCMotorSim liftSim;
  private final DCMotorSim rotateSim;

  private double liftAppliedVolts = 0.0;
  private double rotateAppliedVolts = 0.0;

  public ClimbIOSim() {
    liftSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(liftGearbox, 0.02, 1.0), liftGearbox);
    rotateSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rotateGearbox, 0.004, 1.0), rotateGearbox);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    liftSim.setInputVoltage(MathUtil.clamp(liftAppliedVolts, -12.0, 12.0));
    rotateSim.setInputVoltage(MathUtil.clamp(rotateAppliedVolts, -12.0, 12.0));
    liftSim.update(0.02);
    rotateSim.update(0.02);

    inputs.liftConnected = true;
    inputs.liftAppliedVolts = liftAppliedVolts;
    inputs.liftCurrentAmps = Math.abs(liftSim.getCurrentDrawAmps());

    inputs.rotateConnected = true;
    inputs.rotateAppliedVolts = rotateAppliedVolts;
    inputs.rotateCurrentAmps = Math.abs(rotateSim.getCurrentDrawAmps());

    inputs.liftLowerLimit =
        liftAppliedVolts < 0.0 && Math.abs(liftSim.getAngularPositionRad()) > 10.0;
    inputs.liftUpperLimit =
        liftAppliedVolts > 0.0 && Math.abs(liftSim.getAngularPositionRad()) > 10.0;
    inputs.rotateMinLimit =
        rotateAppliedVolts < 0.0 && Math.abs(rotateSim.getAngularPositionRad()) > 10.0;
    inputs.rotateMaxLimit =
        rotateAppliedVolts > 0.0 && Math.abs(rotateSim.getAngularPositionRad()) > 10.0;
  }

  @Override
  public void setLiftOpenLoop(double volts) {
    liftAppliedVolts = volts;
  }

  @Override
  public void setRotateOpenLoop(double volts) {
    rotateAppliedVolts = volts;
  }
}
