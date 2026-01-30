// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Climb extends VirtualSubsystem {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Climb IO disconnected", AlertType.kWarning);

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    disconnectedAlert.set(!(inputs.liftConnected || inputs.rotateConnected));
  }

  @Override
  public void simulationPeriodic() {}

  private void startLiftUp() {
    io.setLiftOpenLoop(defaultLiftVoltage);
  }

  private void startLiftDown() {
    io.setLiftOpenLoop(-defaultLiftVoltage);
  }

  private void stopLift() {
    io.setLiftOpenLoop(0.0);
  }

  private void startRotateForward() {
    io.setRotateOpenLoop(defaultRotateVoltage);
  }

  private void startRotateReverse() {
    io.setRotateOpenLoop(-defaultRotateVoltage);
  }

  private void stopRotate() {
    io.setRotateOpenLoop(0.0);
  }

  public void stop() {
    stopLift();
    stopRotate();
  }

  public Command liftUpCommand() {
    return Commands.race(
        Commands.runEnd(this::startLiftUp, this::stopLift, this),
        Commands.waitUntil(() -> inputs.liftUpperLimit || !inputs.liftConnected));
  }

  public Command liftDownCommand() {
    return Commands.race(
        Commands.runEnd(this::startLiftDown, this::stopLift, this),
        Commands.waitUntil(() -> inputs.liftLowerLimit || !inputs.liftConnected));
  }

  public Command rotateForwardCommand() {
    return Commands.race(
        Commands.runEnd(this::startRotateForward, this::stopRotate, this),
        Commands.waitUntil(() -> inputs.rotateMaxLimit || !inputs.rotateConnected));
  }

  public Command rotateReverseCommand() {
    return Commands.race(
        Commands.runEnd(this::startRotateReverse, this::stopRotate, this),
        Commands.waitUntil(() -> inputs.rotateMinLimit || !inputs.rotateConnected));
  }
}
