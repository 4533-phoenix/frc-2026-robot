// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for the vision subsystem input/output abstraction.
 *
 * <p>This interface allows for interchangeable vision hardware (e.g., Limelight, PhotonVision) and
 * comprehensive simulation support by standardizing how pose data is retrieved and how robot state
 * is broadcasted back to the vision pipeline.
 */
public interface VisionIO {
  /** Contains all of the inputs received from the vision hardware. */
  @AutoLog
  public static class VisionIOInputs {
    /** Whether the native vision processing server is loaded and running. */
    public boolean serverLoaded = false;

    /** Array of robot poses estimated by the vision system. */
    public Pose2d[] visionPoses = new Pose2d[0];
    /** Array of timestamps (seconds) corresponding to when each pose was captured. */
    public double[] timestamps = new double[0];
    /** Array of standard deviations for the X axis estimate (meters). */
    public double[] stdDevX = new double[0];
    /** Array of standard deviations for the Y axis estimate (meters). */
    public double[] stdDevY = new double[0];
    /** Array of standard deviations for the rotation estimate (radians). */
    public double[] stdDevRot = new double[0];

    /** Array of IDs corresponding to the camera that produced each pose estimate. */
    public int[] cameraIds = new int[0];
    /** Array of the number of AprilTags used to calculate each pose estimate. */
    public int[] tagCounts = new int[0];
  }

  /**
   * Updates the set of loggable inputs with the latest data from the vision system.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * Broadcasts the current robot heading to the vision system to assist in 3D pose estimation.
   *
   * @param heading The current robot heading.
   */
  public default void broadcastRobotHeading(Rotation2d heading) {}
}
