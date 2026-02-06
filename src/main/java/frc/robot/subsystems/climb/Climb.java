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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
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

  public void startLiftUp() {
    io.setLiftOpenLoop(defaultLiftVoltage);
  }

  public void startLiftDown() {
    io.setLiftOpenLoop(-defaultLiftVoltage);
  }

  public void stopLift() {
    io.setLiftOpenLoop(0.0);
  }

  public void startRotateForward() {
    io.setRotateOpenLoop(defaultRotateVoltage);
  }

  public void startRotateReverse() {
    io.setRotateOpenLoop(-defaultRotateVoltage);
  }

  public void stopRotate() {
    io.setRotateOpenLoop(0.0);
  }

  public void stop() {
    stopLift();
    stopRotate();
  }

  public boolean liftUpperLimit() {
    return inputs.liftUpperLimit || !inputs.liftConnected;
  }

  public boolean liftLowerLimit() {
    return inputs.liftLowerLimit || !inputs.liftConnected;
  }

  public boolean rotateForwardLimit() {
    return inputs.rotateMaxLimit || !inputs.rotateConnected;
  }

  public boolean rotateReverseLimit() {
    return inputs.rotateMinLimit || !inputs.rotateConnected;
  }
}
