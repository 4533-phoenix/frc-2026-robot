// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for the vision service input/output abstraction.
 *
 * <p>This interface allows for interchangeable vision hardware (e.g., Limelight, PhotonVision) and
 * comprehensive simulation support by standardizing how pose data is retrieved and how robot state
 * is broadcasted back to the vision pipeline.
 */
public interface VisionIO {
  /** Contains all of the inputs received from the vision hardware. */
  @AutoLog
  public static class VisionIOInputs {
    /** The robot poses corresponding to each vision measurement. */
    public Pose2d[] visionPoses = new Pose2d[0];

    /** The timestamps of each vision measurement in seconds. */
    public double[] timestamps = new double[0];

    /** The ID of the camera that produced each vision measurement. */
    public int[] cameraIds = new int[0];

    /** The number of AprilTags seen in each vision measurement. */
    public int[] tagCounts = new int[0];

    /** The standard deviation of the x position of each vision measurement. */
    public double[] stdDevXs = new double[0];

    /** The standard deviation of the y position of each vision measurement. */
    public double[] stdDevYs = new double[0];

    /** The standard deviation of the rotation of each vision measurement. */
    public double[] stdDevRots = new double[0];
  }

  /**
   * Updates the set of loggable inputs with the latest data from the vision system.
   *
   * @param inputs The inputs object to update.
   */
  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * Broadcasts the high frequency 6-DOF IMU state to the vision coprocessor.
   *
   * @param timestampSec The timestamp of the measurement in seconds.
   * @param compRoll The current roll position in radians.
   * @param compPitch The current pitch position in radians.
   * @param compYaw The current yaw position in radians.
   * @param rollVelRadPerSec The current roll velocity in radians per second.
   * @param pitchVelRadPerSec The current pitch velocity in radians per second.
   * @param yawVelRadPerSec The current yaw velocity in radians per second.
   */
  public default void broadcastTelemetry(
      double timestampSec,
      double compRoll,
      double compPitch,
      double compYaw,
      double rollVelRadPerSec,
      double pitchVelRadPerSec,
      double yawVelRadPerSec) {}
}
