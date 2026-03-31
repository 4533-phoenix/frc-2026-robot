// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.vision;

import frc.lib.IMUState;
import frc.robot.services.vision.Vision.VisionObservation;
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
    /** The latest set of vision observations, including timestamps. */
    public VisionObservation[] observations = new VisionObservation[0];
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
   * @param imuState The latest snapshot of the robot's IMU.
   */
  public default void broadcastTelemetry(IMUState imuState) {}
}
