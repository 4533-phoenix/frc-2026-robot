// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final ArmFeedforward feedforward = new ArmFeedforward(armKs, armKg, armKv, armKa);
  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              armMaxVelocity.in(RadiansPerSecond),
              armMaxAcceleration.in(RadiansPerSecondPerSecond)));
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private double lastTime = 0.0;

  private final Alert armDisconnectedAlert =
      new Alert("Intake arm motor disconnected", AlertType.kWarning);
  private final Alert spinnerDisconnectedAlert =
      new Alert("Intake spinner motor disconnected", AlertType.kWarning);

  public Intake(IntakeIO io) {
    this.io = io;
    lastTime = Timer.getFPGATimestamp();
    setpoint = new TrapezoidProfile.State(armRetractedPosition.in(Radians), 0.0);
    goal = new TrapezoidProfile.State(armRetractedPosition.in(Radians), 0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    armDisconnectedAlert.set(!inputs.armConnected);
    spinnerDisconnectedAlert.set(!inputs.spinnerConnected);

    // Calculate next setpoint using FPGA timestamp for actual loop dt
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    // Fallback to 20ms for non-positive or extremely large dt; clamp to a reasonable max
    if (dt <= 0.0 || Double.isNaN(dt) || dt > 0.5) {
      dt = 0.020;
    }
    lastTime = now;
    setpoint = profile.calculate(Math.min(dt, 0.1), setpoint, goal);

    // Calculate feedforward
    double feedforwardOutput = feedforward.calculate(setpoint.position, setpoint.velocity);

    // Send to SparkMax (Position PID on SparkMax, Profile on RIO)
    io.setArmPosition(Radians.of(setpoint.position), Volts.of(feedforwardOutput));

    Logger.recordOutput("Intake/SetpointPositionDeg", Math.toDegrees(setpoint.position));
    Logger.recordOutput("Intake/GoalPositionDeg", Math.toDegrees(goal.position));
    Logger.recordOutput("Intake/FeedforwardVolts", feedforwardOutput);
  }

  /** Deploy the intake arm and spin the rollers. */
  public void deploy() {
    goal = new TrapezoidProfile.State(armDeployedPosition.in(Radians), 0.0);
    io.setSpinnerVoltage(spinnerIntakeVoltage);
  }

  /** Retract the intake arm and stop the rollers. */
  public void retract() {
    goal = new TrapezoidProfile.State(armRetractedPosition.in(Radians), 0.0);
    io.setSpinnerVoltage(Volts.of(0.0));
  }

  /** Stop everything (arm holds position at retracted, spinner off). */
  public void stop() {
    goal = new TrapezoidProfile.State(armRetractedPosition.in(Radians), 0.0);
    io.setSpinnerVoltage(Volts.of(0.0));
  }

  @AutoLogOutput(key = "Intake/ArmPositionDeg")
  public double getArmPositionDeg() {
    return inputs.armPosition.in(Degrees);
  }
}
