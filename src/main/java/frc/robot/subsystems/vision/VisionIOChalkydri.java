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
 * AprilTag pose estimations. It maps raw data from a shared direct byte buffer into WPILib geometry
 * objects, utilizing zero-allocation patterns to maintain high performance.
 */
public class VisionIOChalkydri implements VisionIO {
  /** The Whacknet instance for communication with the Chalkydri coprocessors. */
  private final Whacknet whacknet;

  /** Creates a new VisionIOChalkydri and starts the native vision server. */
  public VisionIOChalkydri() {
    whacknet = Whacknet.getInstance();
    whacknet.start(serverPort);
  }

  /**
   * Updates inputs by reading the latest packets from the native library and mapping them to
   * loggable Java objects.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.serverLoaded = whacknet.isLoaded();

    // Read packets from shared memory and get the count of observations
    int count = whacknet.readPackets();

    // Resize arrays to match the number of packets if necessary
    if (inputs.visionPoses.length != count) {
      inputs.visionPoses = new Pose2d[count];
      inputs.timestamps = new double[count];
      inputs.cameraIds = new int[count];
      inputs.tagCounts = new int[count];

      // Allocating three 1D arrays is significantly faster than allocating a 2D array
      inputs.stdDevX = new double[count];
      inputs.stdDevY = new double[count];
      inputs.stdDevRot = new double[count];
    }

    // Populate loggable inputs using the zero-allocation Flyweight view
    whacknet.forEachPacket(
        (packet, i) -> {
          inputs.visionPoses[i] = packet.getPose2d();

          // Convert microsecond FPGA time to seconds
          inputs.timestamps[i] = packet.getTimestamp() * 1.0e-6;
          inputs.cameraIds[i] = packet.getCameraId();
          inputs.tagCounts[i] = packet.getNumTags();

          inputs.stdDevX[i] = packet.getStdX();
          inputs.stdDevY[i] = packet.getStdY();
          inputs.stdDevRot[i] = packet.getStdRot();
        });
  }

  /**
   * Broadcasts the current robot heading back to the native vision pipeline to assist with pose
   * estimation.
   *
   * @param heading The current robot heading in radians, normalized.
   */
  @Override
  public void broadcastRobotHeading(Rotation2d heading) {
    whacknet.broadcast(MathUtil.angleModulus(heading.getRadians()));
  }
}
