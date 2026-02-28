// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform2d;
import java.util.Map;

/**
 * Hardware and tuning constants for the vision subsystem.
 *
 * <p>Contains configurations for cameras, including their positions relative to the robot,
 * field-of-view limits, and maximum detection ranges.
 */
public class VisionConstants {
  /**
   * Defines the physical and operational parameters of a camera.
   *
   * @param name The human-readable name of the camera.
   * @param robotToCamera The transform from the robot center to the camera lens.
   * @param fovDegrees The horizontal field of view of the camera in degrees.
   * @param maxRangeMeters The maximum distance the camera can reliably detect AprilTags.
   */
  public record CameraConfig(
      String name, Transform2d robotToCamera, double fovDegrees, double maxRangeMeters) {}

  /**
   * Map of Camera ID to its configuration. IDs should match those used in the native vision
   * pipeline.
   */
  public static final Map<Integer, CameraConfig> cameraMap =
      Map.of(1, new CameraConfig("TestCamera", Transform2d.kZero, 90.0, 10.0));

  /** Time in seconds before a camera is considered offline if no data is received. */
  public static final double offlineTimeoutSeconds = 1.0;
  /** Port for communication between Java and the native vision server. */
  public static final int serverPort = 7001;
}
