// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util.sparktap;

import edu.wpi.first.wpilibj.RobotBase;
import java.nio.ByteBuffer;

/** Low-level JNI hooks for the SparkTap CAN interceptor. */
public class SparkTapJNI {
  static {
    if (RobotBase.isReal()) {
      try {
        System.loadLibrary("sparktap");
      } catch (UnsatisfiedLinkError e) {
        System.err.println("[SparkTap-java] Native library failed to load!");
      }
    }
  }

  /**
   * Initializes the native stream and returns the shared memory buffer.
   *
   * @return Direct ByteBuffer mapping to the native status table.
   */
  public static native ByteBuffer init();

  /**
   * Blocks the current thread until the specified motor frame is updated.
   *
   * @param deviceId CAN ID of the motor (0-63).
   * @param frameIdx Status frame index (0-6).
   */
  public static native void waitForFrame(int deviceId, int frameIdx);
}
