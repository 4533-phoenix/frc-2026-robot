// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import frc.robot.subsystems.vision.Vision.VisionObservation;
import frc.robot.util.Whacknet;

/**
 * Real IO implementation for the vision subsystem using the 'Whacknet' JNI wrapper.
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
    whacknet.start(SERVER_RPORT, SERVER_BPORT);
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
    int count = whacknet.readPackets();

    // Create a single array of our struct
    inputs.observations = new VisionObservation[count];

    whacknet.forEachPacket(
        (packet, i) -> {
          inputs.observations[i] =
              new VisionObservation(
                  packet.getPose2d(),
                  packet.getTimestamp() * 1.0e-6,
                  packet.getCameraId(),
                  packet.getNumTags(),
                  packet.getStdX(),
                  packet.getStdY(),
                  packet.getStdRot());
        });
  }
}
