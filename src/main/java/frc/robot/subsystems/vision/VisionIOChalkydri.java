// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.Whacknet;

/**
 * Real IO implementation for the vision subsystem using the 'Whacknet' JNI wrapper.
 *
 * <p>This implementation communicates with a high-performance native vision pipeline to retrieve
 * AprilTag pose estimations. It maps raw data from a shared direct byte buffer into WPILib
 * geometry objects.
 */
public class VisionIOChalkydri implements VisionIO {
  private final Whacknet vision;

  /** Creates a new VisionIOChalkydri and starts the native vision server. */
  public VisionIOChalkydri() {
    vision = Whacknet.getInstance();
    vision.start(serverPort);
  }

  /**
   * Updates inputs by reading the latest packets from the native library and mapping them to
   * loggable Java objects.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.serverLoaded = vision.isLoaded();
    // Read packets from shared memory and get the count of observations
    int count = vision.readPackets();

    // Resize arrays to match the number of packets if necessary
    if (inputs.visionPoses == null || inputs.visionPoses.length != count) {
      inputs.visionPoses = new Pose2d[count];
      inputs.timestamps = new double[count];
      inputs.cameraIds = new int[count];
      inputs.tagCounts = new int[count];
      inputs.stdDevs = new double[count][3];
    }

    // Populate loggable inputs from the direct byte buffer
    for (int i = 0; i < count; i++) {
      inputs.visionPoses[i] =
          new Pose2d(
              vision.getX(i),
              vision.getY(i),
              Rotation2d.fromRadians(vision.getRot(i)));
      // Convert microsecond FPGA time to seconds
      inputs.timestamps[i] = vision.getTimestamp(i) * 1.0e-6;
      inputs.cameraIds[i] = vision.getCameraId(i);
      inputs.tagCounts[i] = vision.getNumTags(i);

      inputs.stdDevs[i][0] = vision.getStdX(i);
      inputs.stdDevs[i][1] = vision.getStdY(i);
      inputs.stdDevs[i][2] = vision.getStdRot(i);
    }
  }

  /**
   * Broadcasts the current robot heading back to the native vision pipeline to assist with
   * pose estimation.
   *
   * @param heading The current robot heading in radians, normalized.
   */
  @Override
  public void broadcastRobotHeading(double heading) {
    vision.broadcast(MathUtil.angleModulus(heading));
  }
}
