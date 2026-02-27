// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform2d;
import java.util.Map;

public class VisionConstants {
  public record CameraConfig(
      String name, Transform2d robotToCamera, double fovDegrees, double maxRangeMeters) {}

  public static final Map<Integer, CameraConfig> cameraMap =
      Map.of(1, new CameraConfig("TestCamera", Transform2d.kZero, 90.0, 10.0));
  public static final double offlineTimeoutSeconds = 1.0;
  public static final int serverPort = 7001;
}
