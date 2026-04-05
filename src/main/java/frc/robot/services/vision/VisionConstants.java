// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.util.Map;

/**
 * Hardware and tuning constants for the vision service.
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
  public static final Map<Integer, CameraConfig> CAMERA_MAP =
      Map.of(
          1,
          new CameraConfig(
              "Front",
              new Transform3d(
                  Meters.of(-0.105),
                  Meters.of(-0.0075),
                  Meters.of(49.590851),
                  new Rotation3d(Degrees.of(180.0), Degrees.of(-5.0), Degrees.of(0.0))),
              92.0,
              10.0),
          2,
          new CameraConfig(
              "Back Left",
              new Transform3d(
                  Meters.of(-0.3068556),
                  Meters.of(0.1930089),
                  Meters.of(0.23971232),
                  new Rotation3d(Degrees.of(90.0), Degrees.of(-20.0), Degrees.of(133.0))),
              92.0,
              10.0),
          3,
          new CameraConfig(
              "Back Right",
              new Transform3d(
                  Meters.of(-0.2916262),
                  Meters.of(-0.2806894),
                  Meters.of(0.25494719),
                  new Rotation3d(Degrees.of(0.0), Degrees.of(-20.0), Degrees.of(-135.0))),
              92.0,
              10.0));

  /** Time in seconds before a camera is considered offline if no data is received. */
  public static final Time OFFLINE_TIMEOUT = Seconds.of(1.0);

  /** Port for communication between Java and the native vision server. */
  public static final int SERVER_RPORT = 7001;

  /** Port for the native vision server to broadcast heading information. */
  public static final int SERVER_BPORT = 7002;

  /**
   * Standard deviations for vision measurements when the data is not valid or should be ignored.
   */
  public static final Matrix<N3, N1> NO_STD_DEVS = VecBuilder.fill(0, 0, 0);

  /** Standard deviations for vision measurements when only one AprilTag is visible. */
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(2, 2, 4.0);

  /**
   * Standard deviations for vision measurements when multiple AprilTags are visible (higher
   * confidence).
   */
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);

  /** The maximum allowed ambiguity for a single-tag detection to be considered valid. */
  public static final double AMBIGUITY_CUTOFF = 0.05;

  /** The maximum distance from the camera to a tag for a single-tag detection to be trusted. */
  public static final Distance SINGLE_TAG_POSE_CUTOFF = Meters.of(4.0);

  /** Sentinel value indicating that no ambiguity calculation is applicable (e.g., multi-tag). */
  public static final int NO_AMBIGUITY = -100;

  /** The AprilTag field layout representing the positions of tags on the current year's field. */
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /** Whether or not to broadcast heading information. */
  public static final boolean BROADCAST_HEADING = true;
}
