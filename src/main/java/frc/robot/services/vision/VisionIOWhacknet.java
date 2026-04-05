// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.vision;

import static frc.robot.services.vision.VisionConstants.*;

import frc.lib.IMUState;
import frc.lib.lowlevel.Whacknet;
import frc.robot.services.vision.Vision.VisionObservation;

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

  /** Creates a new VisionIOChalkydri and starts the native vision server. */
  public VisionIOWhacknet() {
    whacknet = Whacknet.getInstance();
    whacknet.startServer(SERVER_RPORT);
  }

  /**
   * Updates inputs by reading the latest packets from the native library and mapping them to
   * loggable Java objects.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    int count = whacknet.readPackets();

    // Create a single array of our struct
    inputs.observations = new VisionObservation[count];

    whacknet.forEachPacket(
        (packet, i) -> {
          double timestampSeconds = packet.getTimestamp() / 1_000_000.0;

          inputs.observations[i] =
              new VisionObservation(
                  packet.getPose2d(),
                  timestampSeconds,
                  packet.getCameraId(),
                  packet.getNumTags(),
                  packet.getStdX(),
                  packet.getStdY(),
                  packet.getStdRot());
        });
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
