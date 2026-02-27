// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for processing vision data, specifically from AprilTag cameras.
 *
 * <p>Receives raw camera detections, filters them based on quality (tag count), and feeds valid
 * measurements into the Drive subsystem's pose estimator to refine the robot's field position.
 * Also monitors camera health and status.
 */
public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final Drive drive;

  // Track the last timestamp each camera was seen to detect offline cameras
  private final Map<Integer, Double> lastTimestampMap = new HashMap<>();
  private final Map<Integer, Alert> alertMap = new HashMap<>();

  // Pre-allocated standard deviation vector to avoid allocations in periodic()
  private final Matrix<N3, N1> stdVector = VecBuilder.fill(0, 0, 0);

  // Pre-computed log paths to avoid string concatenation per cycle
  private final Map<Integer, String> logPaths = new HashMap<>();

  /**
   * Creates a new Vision subsystem.
   *
   * @param io The abstraction layer for the vision hardware (e.g., Limelight, PhotonVision).
   * @param drive The Drive subsystem instance for updating pose estimates.
   */
  public Vision(VisionIO io, Drive drive) {
    this.io = io;
    this.drive = drive;

    // Initialize maps for tracking camera status based on Constants
    for (var entry : cameraMap.entrySet()) {
      lastTimestampMap.put(entry.getKey(), Timer.getTimestamp());
      alertMap.put(
          entry.getKey(),
          new Alert("Vision: Camera '" + entry.getValue() + "' is offline!", AlertType.kWarning));
      logPaths.put(entry.getKey(), "Vision/CameraStatus/" + entry.getValue());
    }
  }

  /**
   * Processes vision measurements, filters invalid data, updates the drive pose estimator,
   * and checks camera status.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    // Broadcast current robot heading to native vision pipeline for improved pose estimation
    io.broadcastRobotHeading(drive.getRotation().getRadians());

    double currentTime = Timer.getTimestamp();

    // Process all detections received this frame
    for (int i = 0; i < inputs.visionPoses.length; i++) {
      // Quality Filter: Ignore measurements that detected zero or one AprilTag
      // Single-tag detections are often unreliable for field position
      if (inputs.tagCounts != null && inputs.tagCounts.length > i && inputs.tagCounts[i] <= 1) {
        continue;
      }

      int id = inputs.cameraIds[i];
      lastTimestampMap.put(id, currentTime);

      // Update Drive Subsystem with refined pose
      // Reuse pre-allocated vector instead of creating new one every cycle for performance
      stdVector.set(0, 0, inputs.stdDevs[i][0]);
      stdVector.set(1, 0, inputs.stdDevs[i][1]);
      stdVector.set(2, 0, inputs.stdDevs[i][2]);
      drive.addVisionMeasurement(inputs.visionPoses[i], inputs.timestamps[i], stdVector);
    }

    // Camera Status Monitoring: Check for offline cameras
    for (int id : cameraMap.keySet()) {
      double lastSeen = lastTimestampMap.getOrDefault(id, 0.0);
      boolean isOffline = (currentTime - lastSeen) > offlineTimeoutSeconds;

      // Update Alerts for drivers and log status
      alertMap.get(id).set(isOffline);
      Logger.recordOutput(logPaths.get(id), !isOffline);
    }
  }
}
