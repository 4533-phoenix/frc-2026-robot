// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Map;

public class VisionConstants {
  /** Map of Camera IDs (from C) to Human Readable Names */
  public static final Map<Long, String> CAMERA_MAP =
      Map.of(
          0L, "FrontLeft",
          1L, "FrontRight",
          2L, "BackCenter");

  /** Time in seconds before we consider a camera offline */
  public static final double OFFLINE_TIMEOUT_SECONDS = 1.0;

  /** What port the vision server listens on */
  public static final int SERVER_PORT = 7001;
}
