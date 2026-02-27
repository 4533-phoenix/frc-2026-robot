// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for the robot's climb mechanism.
 *
 * <p>Handles controlling the lift motor voltage and monitoring limit switches to prevent
 * over-extension or damage to the mechanism.
 */
public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Climb IO disconnected", AlertType.kWarning);

  /**
   * Creates a new Climb subsystem.
   *
   * @param io The abstraction layer for the climb hardware.
   */
  public Climb(ClimbIO io) {
    this.io = io;
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    disconnectedAlert.set(!inputs.connected);
  }

  @Override
  public void simulationPeriodic() {}

  /** Moves the lift mechanism up at the default voltage. */
  public void startLiftUp() {
    io.setLiftVoltage(defaultLiftVoltage);
  }

  /** Moves the lift mechanism down at the default voltage. */
  public void startLiftDown() {
    io.setLiftVoltage(defaultLiftVoltage.unaryMinus());
  }

  /** Stops the lift mechanism. */
  public void stopLift() {
    io.setLiftVoltage(Volts.of(0.0));
  }

  /** Stops all climb mechanism components. */
  public void stop() {
    stopLift();
  }

  /**
   * Checks if the lift has reached its upper limit.
   *
   * @return True if upper limit switch is pressed or hardware is disconnected.
   */
  public boolean liftUpperLimit() {
    return inputs.upperLimit || !inputs.connected;
  }

  /**
   * Checks if the lift has reached its lower limit.
   *
   * @return True if lower limit switch is pressed or hardware is disconnected.
   */
  public boolean liftLowerLimit() {
    return inputs.lowerLimit || !inputs.connected;
  }
}
