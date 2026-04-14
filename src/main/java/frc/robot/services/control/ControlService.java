// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.control;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import frc.lib.monitor.MonitoredBaseService;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Abstract base class for control services. Handles chooser logic, standardized logging, rumble
 * hardware updates, and disconnect alerts.
 *
 * @param <P> The ControlProfile type
 * @param <IO> The IO interface type
 * @param <IN> The AutoLogged inputs type
 */
public abstract class ControlService<P extends ControlProfile, IO, IN extends LoggableInputs>
    extends MonitoredBaseService {

  /** Dashboard chooser for profiles. */
  protected final LoggedDashboardChooser<P> chooser;

  /** IO interface for hardware interaction. */
  protected final IO io;

  /** Inputs object for logging and state. */
  protected final IN inputs;

  private final String serviceName;
  private final Alert connectionAlert;

  private double lastLeftRumble = 0.0;
  private double lastRightRumble = 0.0;

  /**
   * Constructs a ControlService.
   *
   * @param serviceName The name of the service.
   * @param io The IO interface for hardware interaction.
   * @param inputs The inputs object for logging and state.
   * @param chooser The dashboard chooser for profiles.
   */
  public ControlService(String serviceName, IO io, IN inputs, LoggedDashboardChooser<P> chooser) {
    this.serviceName = serviceName;
    this.io = io;
    this.inputs = inputs;
    this.chooser = chooser;

    this.connectionAlert = new Alert(serviceName + " controller disconnected", AlertType.kError);
  }

  @Override
  public void update() {
    P profile = chooser.get();

    // Handle null/missing profile
    if (profile == null) {
      connectionAlert.set(true);
      resetRumble(null);
      return;
    }

    // Update IO, Log Inputs, Log Active Profile
    updateInputs(io, inputs, profile);
    Logger.processInputs(serviceName, inputs);

    // Update connection health alert
    connectionAlert.set(!profile.isConnected());

    // Update physical hardware (Rumble)
    updateRumble(profile);
  }

  /** Manages rumble logic: stops rumbling when disabled and only sends updates if values change. */
  private void updateRumble(P profile) {
    GenericHID hid = profile.getHID();
    if (hid == null) {
      lastLeftRumble = 0.0;
      lastRightRumble = 0.0;
      return;
    }

    double targetLeft = 0.0;
    double targetRight = 0.0;

    // Only allow rumble if the robot is enabled
    if (DriverStation.isEnabled()) {
      targetLeft = profile.getLeftRumble();
      targetRight = profile.getRightRumble();
    }

    // Only send updates to the hardware if the value has changed
    if (targetLeft != lastLeftRumble) {
      hid.setRumble(GenericHID.RumbleType.kLeftRumble, targetLeft);
      lastLeftRumble = targetLeft;
    }

    if (targetRight != lastRightRumble) {
      hid.setRumble(GenericHID.RumbleType.kRightRumble, targetRight);
      lastRightRumble = targetRight;
    }
  }

  /** Forces rumble to stop and resets tracking variables. */
  private void resetRumble(GenericHID hid) {
    if (hid != null) {
      hid.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
      hid.setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }
    lastLeftRumble = 0.0;
    lastRightRumble = 0.0;
  }

  /**
   * Subclasses must map the profile methods to the inputs object.
   *
   * @param io The IO interface for hardware interaction.
   * @param inputs The inputs object for logging and state.
   * @param profile The control profile.
   */
  protected abstract void updateInputs(IO io, IN inputs, P profile);

  @Override
  public boolean isHealthy() {
    return !connectionAlert.get();
  }

  @Override
  public void clearFaults() {}
}
