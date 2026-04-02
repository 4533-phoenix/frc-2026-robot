// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.services.vision.Vision;
import frc.robot.services.vision.VisionIOPhoton;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIONavX;
import frc.robot.subsystems.drive.module.ModuleIO;

/** Declares the robot's subsystems, operator interface devices, and command bindings. */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * <p>Configures IO implementations based on the current mode (Real, Sim, or Replay).
   */
  public RobotContainer() {
    drive =
        new Drive(
            new GyroIONavX(),
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    vision = new Vision(new VisionIOPhoton());
    // Wire up the data flow from vision to drive and drive to vision
    drive.setIMUHighFreqConsumer(vision::broadcastTelemetry);
    drive.setAccuratePoseSupplier(vision::getBestSeedPose);
    vision.setVisionMeasurementConsumer(drive::addVisionMeasurement);

    // drive.setPose(Pose2d.kZero);
    CommandXboxController controller = new CommandXboxController(0);
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.setPose(Pose2d.kZero);
                }));
  }
}
