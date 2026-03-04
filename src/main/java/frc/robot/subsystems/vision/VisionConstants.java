// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
      String name, Transform3d robotToCamera, double fovDegrees, double maxRangeMeters) {}

  /**
   * Map of Camera ID to its configuration. IDs should match those used in the native vision
   * pipeline.
   */
  public static final Map<Integer, CameraConfig> cameraMap =
      Map.of(
          1,
          new CameraConfig(
              "FrontLeft",
              new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(45))),
              90.0,
              10.0),
          2,
          new CameraConfig(
              "BackLeft",
              new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(135))),
              90.0,
              10.0),
          3,
          new CameraConfig(
              "BackRight",
              new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(225))),
              90.0,
              10.0),
          4,
          new CameraConfig(
              "FrontRight",
              new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(315))),
              90.0,
              10.0));

  /** Time in seconds before a camera is considered offline if no data is received. */
  public static final double offlineTimeoutSeconds = 1.0;
  /** Port for communication between Java and the native vision server. */
  public static final int serverPort = 7001;

  public static final Matrix<N3, N1> noStdDevs = VecBuilder.fill(0, 0, 0);
  public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, Double.MAX_VALUE);
  public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
  public static final Matrix<N3, N1> tagStdDevs = VecBuilder.fill(0, 0, Double.MAX_VALUE);

  public static final double ambiguityCutoff = 0.05;
  public static final double singleTagPoseCutoffMeters = 4.0;
  public static final int noAmbiguity = -100;

  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
}
