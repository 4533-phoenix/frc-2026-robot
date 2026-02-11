// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean serverLoaded = false;

    public Pose2d[] visionPoses = new Pose2d[] {};
    public double[] timestamps = new double[] {};
    public double[][] stdDevs = new double[][] {};
    public int[] cameraIds = new int[] {};
    public int[] tagCounts = new int[] {};
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void broadcastRobotHeading(double heading) {}
}
