// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.vision;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.services.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.IMUState;
import frc.lib.monitor.MonitoredBaseService;
import java.util.Collections;
import org.littletonrobotics.junction.Logger;

/**
 * Service for processing vision data, specifically from AprilTag cameras.
 *
 * <p>Receives raw camera detections, filters them based on quality (tag count), and feeds valid
 * measurements into the Drive subsystem's pose estimator to refine the robot's field position. Also
 * monitors camera health and status using highly optimized zero-allocation data structures.
 */
public class Vision extends MonitoredBaseService {
  /** A record representing a single vision observation. */
  public record VisionObservation(
      Pose2d visionPose,
      double timestamp,
      int cameraId,
      int tagCount,
      double stdDevX,
      double stdDevY,
      double stdDevRot) {}

  /** A functional interface for consuming vision measurements. */
  @FunctionalInterface
  public interface VisionMeasurementConsumer {
    /**
     * Accepts a new vision measurement for processing.
     *
     * @param visionPose The estimated pose from the vision system.
     * @param timestamp The timestamp of the measurement.
     * @param stdDevs A 3x1 matrix containing the standard deviations for x, y, and rotation.
     */
    void accept(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs);
  }

  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  // State variables
  private VisionMeasurementConsumer measurementConsumer = null;

  // We size the arrays based on the highest camera ID in the map to ensure direct index mapping.
  private final int maxCameraId;
  private final double[] lastTimestamps;
  private final Alert[] alerts;
  private final String[] logPaths;
  private final String[] seenPaths;
  private final boolean[] cameraActiveFlags;

  // Pre-allocated standard deviation vector to avoid allocations in periodic()
  private final Matrix<N3, N1> stdVector = VecBuilder.fill(0, 0, 0);

  /**
   * Creates a new Vision service.
   *
   * @param io The abstraction layer for the vision hardware.
   */
  public Vision(VisionIO io) {
    this.io = io;

    // Determine the max array size needed to sequentially store camera data
    maxCameraId = CAMERA_MAP.isEmpty() ? 0 : Collections.max(CAMERA_MAP.keySet());

    lastTimestamps = new double[maxCameraId + 1];
    alerts = new Alert[maxCameraId + 1];
    logPaths = new String[maxCameraId + 1];
    seenPaths = new String[maxCameraId + 1];
    cameraActiveFlags = new boolean[maxCameraId + 1];

    // Initialize arrays for tracking camera status based on Constants
    for (var entry : CAMERA_MAP.entrySet()) {
      int id = entry.getKey();
      cameraActiveFlags[id] = true;
      lastTimestamps[id] = Timer.getTimestamp();
      alerts[id] = new Alert(entry.getValue().name() + " camera offline", AlertType.kWarning);
      logPaths[id] = "Vision/CameraStatus/" + entry.getValue().name();
      seenPaths[id] = "Vision/CameraSeen/" + entry.getValue().name();
    }
  }

  /**
   * Forwards a high frequency IMU packet to the vision pipeline.
   *
   * @param imuState The current 6-DOF IMU state.
   */
  public void broadcastTelemetry(IMUState imuState) {
    io.broadcastTelemetry(imuState);
  }

  /**
   * Processes vision measurements, filters invalid data, updates the drive pose estimator, and
   * checks camera status.
   */
  @Override
  public void update() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    double currentTime = Timer.getTimestamp();

    // Process all detections received this frame
    for (int i = 0; i < inputs.observations.length; i++) {
      int id = inputs.observations[i].cameraId();
      if (id >= 0 && id <= maxCameraId && cameraActiveFlags[id]) {
        lastTimestamps[id] = currentTime;
      }

      // Single-tag detections are often unreliable for field position
      if (inputs.observations[i].tagCount() <= 1) {
        continue;
      }

      // Update Consumer with refined pose
      stdVector.set(0, 0, inputs.observations[i].stdDevX());
      stdVector.set(1, 0, inputs.observations[i].stdDevY());
      stdVector.set(2, 0, Double.MAX_VALUE);

      if (measurementConsumer != null) {
        measurementConsumer.accept(
            inputs.observations[i].visionPose(), inputs.observations[i].timestamp(), stdVector);
      }
    }

    // Check for offline cameras
    for (int id = 0; id <= maxCameraId; id++) {
      if (!cameraActiveFlags[id]) continue;

      boolean isOffline = (currentTime - lastTimestamps[id]) > OFFLINE_TIMEOUT.in(Seconds);

      // Update Alerts for drivers and log status
      alerts[id].set(isOffline);
      Logger.recordOutput(logPaths[id], !isOffline);
      boolean seen = (currentTime - lastTimestamps[id]) <= OFFLINE_TIMEOUT.in(Seconds);
      Logger.recordOutput(seenPaths[id], seen);
    }
  }

  /**
   * Returns whether or not the service is healthy
   *
   * @return True if the service is healthy, false otherwise.
   */
  public boolean isHealthy() {
    double currentTime = Timer.getTimestamp();
    for (int id = 0; id <= maxCameraId; id++) {
      if (cameraActiveFlags[id]
          && (currentTime - lastTimestamps[id]) <= OFFLINE_TIMEOUT.in(Seconds)) {
        return true;
      }
    }
    return false;
  }

  /** Clear all faults and reset the service. */
  public void clearFaults() {}

  /**
   * Sets the consumer for vision measurements.
   *
   * @param consumer A callback function that will be called with each valid vision measurement,
   *     providing the pose, timestamp, and standard deviations. This allows external systems to
   *     react to new vision data in real-time.
   */
  public void setVisionMeasurementConsumer(VisionMeasurementConsumer consumer) {
    this.measurementConsumer = consumer;
  }
}
