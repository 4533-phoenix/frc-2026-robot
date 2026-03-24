// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroIONavX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOWhacknet;

/** Declares the robot's subsystems, operator interface devices, and command bindings. */
public class RobotContainer {
  // Subsystems
  @SuppressWarnings("unused")
  private final Gyro gyro;

  @SuppressWarnings("unused")
  private final Vision vision;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * <p>Configures IO implementations based on the current mode (Real, Sim, or Replay).
   */
  public RobotContainer() {
    // Instantiate subsystems based on the running mode
    gyro = new Gyro(new GyroIONavX());
    vision = new Vision(new VisionIOWhacknet());
  }
}
