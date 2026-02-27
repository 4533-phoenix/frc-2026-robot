// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          armMotorReduction,
          0.5, // Simulated MOI
          0.5, // Simulated length in meters
          armDeployedPosition.in(Radians), // min angle
          armRetractedPosition.in(Radians), // max angle
          true, // simulate gravity
          armRetractedPosition.in(Radians) // starting angle
          );

  private final FlywheelSim spinnerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.005, spinnerMotorReduction),
          DCMotor.getNEO(1),
          spinnerMotorReduction);

  private final ProfiledPIDController armController =
      new ProfiledPIDController(
          armKp,
          0.0,
          armKd,
          new TrapezoidProfile.Constraints(
              armCruiseVelocity.in(RadiansPerSecond),
              armMaxAcceleration.in(RadiansPerSecondPerSecond)));

  private final PIDController spinnerController = new PIDController(spinnerKp, 0.0, spinnerKd);

  private boolean armClosedLoop = false;
  private boolean spinnerClosedLoop = false;

  private Angle armSetpoint = armRetractedPosition;
  private AngularVelocity spinnerSetpoint = RadiansPerSecond.of(0.0);

  private Voltage armAppliedVoltage = Volts.of(0.0);
  private Voltage spinnerAppliedVoltage = Volts.of(0.0);

  public IntakeIOSim() {
    armController.reset(armRetractedPosition.in(Radians));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (armClosedLoop) {
      double pidVal = armController.calculate(armSim.getAngleRads(), armSetpoint.in(Radians));
      double setpointVel = armController.getSetpoint().velocity;
      double ffVal =
          armKs * Math.signum(setpointVel)
              + armKg * Math.cos(armSim.getAngleRads())
              + armKv * setpointVel;
      armAppliedVoltage = Volts.of(MathUtil.clamp(pidVal + ffVal, -12.0, 12.0));
    } else {
      armAppliedVoltage = Volts.of(0.0);
    }

    if (spinnerClosedLoop) {
      double pidVal =
          spinnerController.calculate(
              spinnerSim.getAngularVelocityRadPerSec(), spinnerSetpoint.in(RadiansPerSecond));
      double ffVal =
          spinnerKs * Math.signum(spinnerSetpoint.in(RadiansPerSecond))
              + spinnerKv * spinnerSetpoint.in(RadiansPerSecond);
      spinnerAppliedVoltage = Volts.of(MathUtil.clamp(pidVal + ffVal, -12.0, 12.0));
    } else {
      spinnerAppliedVoltage = Volts.of(0.0);
    }

    armSim.setInputVoltage(armAppliedVoltage.in(Volts));
    spinnerSim.setInputVoltage(spinnerAppliedVoltage.in(Volts));

    armSim.update(0.02);
    spinnerSim.update(0.02);

    inputs.armConnected = true;
    inputs.armPosition = Radians.of(armSim.getAngleRads());
    inputs.armVelocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());
    inputs.armAppliedVoltage = armAppliedVoltage;
    inputs.armAppliedCurrent = Amps.of(armSim.getCurrentDrawAmps());

    inputs.spinnerConnected = true;
    inputs.spinnerVelocity = RadiansPerSecond.of(spinnerSim.getAngularVelocityRadPerSec());
    inputs.spinnerAppliedVoltage = spinnerAppliedVoltage;
    inputs.spinnerAppliedCurrent = Amps.of(spinnerSim.getCurrentDrawAmps());
  }

  @Override
  public void setArmPosition(Angle angle) {
    armClosedLoop = true;
    armSetpoint = angle;
  }

  @Override
  public void setSpinnerAngularVelocity(AngularVelocity velocity) {
    spinnerClosedLoop = true;
    spinnerSetpoint = velocity;
  }
}
