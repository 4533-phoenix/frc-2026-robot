// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.vision;

import static frc.robot.services.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.IMUState;
import frc.lib.lowlevel.Whacknet;

/**
 * Real IO implementation for the vision service using Whacknet.
 *
 * <p>This implementation communicates with a high-performance native vision pipeline to retrieve
 * AprilTag pose estimations. It maps raw data from a shared direct byte buffer into WPILib geometry
 * objects, utilizing zero-allocation patterns to maintain high performance.
 */
public class VisionIOWhacknet implements VisionIO {
  /** The Whacknet instance for communication with the Chalkydri coprocessors. */
  private final Whacknet whacknet;

  // Pre-allocated array pools to avoid GC overhead
  private final Pose2d[][] posePool = new Pose2d[65][];
  private final double[][] timestampPool = new double[65][];
  private final int[][] cameraIdPool = new int[65][];
  private final int[][] tagCountPool = new int[65][];
  private final double[][] stdDevXPool = new double[65][];
  private final double[][] stdDevYPool = new double[65][];
  private final double[][] stdDevRotPool = new double[65][];

  /** Creates a new VisionIOWhacknet and starts the native vision server. */
  public VisionIOWhacknet() {
    whacknet = Whacknet.getInstance();
    whacknet.startServer(SERVER_RPORT);

    // Initialize the fixed-size pools for 0 to 64 possible frames
    for (int i = 0; i <= 64; i++) {
      posePool[i] = new Pose2d[i];
      timestampPool[i] = new double[i];
      cameraIdPool[i] = new int[i];
      tagCountPool[i] = new int[i];
      stdDevXPool[i] = new double[i];
      stdDevYPool[i] = new double[i];
      stdDevRotPool[i] = new double[i];
    }
  }

  /**
   * Updates inputs by reading the latest packets from the native library and mapping them to
   * loggable Java objects.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    int count = whacknet.readPackets();

    int safeCount = Math.min(count, 64);

    inputs.visionPoses = posePool[safeCount];
    inputs.timestamps = timestampPool[safeCount];
    inputs.cameraIds = cameraIdPool[safeCount];
    inputs.tagCounts = tagCountPool[safeCount];
    inputs.stdDevXs = stdDevXPool[safeCount];
    inputs.stdDevYs = stdDevYPool[safeCount];
    inputs.stdDevRots = stdDevRotPool[safeCount];

    if (count > 0) {
      whacknet.forEachPacket(
          (packet, i) -> {
            if (i >= safeCount) return;
            inputs.visionPoses[i] =
                packet.getPose2d();
            inputs.timestamps[i] = packet.getTimestamp() / 1_000_000.0;
            inputs.cameraIds[i] = packet.getCameraId();
            inputs.tagCounts[i] = packet.getNumTags();
            inputs.stdDevXs[i] = packet.getStdX();
            inputs.stdDevYs[i] = packet.getStdY();
            inputs.stdDevRots[i] = packet.getStdRot();
          });
    }
  }

  @SuppressWarnings("unused")
  @Override
  public void broadcastTelemetry(IMUState imuState) {
    if (whacknet != null && imuState != null && BROADCAST_HEADING) {
      whacknet.broadcastTelemetry(
          (long) (imuState.timestampSec() * 1.0e6),
          imuState.rollRad(),
          imuState.pitchRad(),
          imuState.yawRad(),
          imuState.rollVelRadPerSec(),
          imuState.pitchVelRadPerSec(),
          imuState.yawVelRadPerSec(),
          VisionConstants.SERVER_BPORT);
    }
  }
}
