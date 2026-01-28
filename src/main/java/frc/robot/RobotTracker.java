// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.PoseObservation;

public class RobotTracker {
  public static Pose2d getGlobalPose() {
    return new Pose2d();
  }

  public static void offerGlobalVisionObservation(PoseObservation observation) {}

  public static void offerConstrainedVisionObservation(PoseObservation observation) {}
}
