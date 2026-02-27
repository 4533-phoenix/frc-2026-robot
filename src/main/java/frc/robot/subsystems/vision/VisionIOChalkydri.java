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

public class VisionIOChalkydri implements VisionIO {
  private final Whacknet vision;

  public VisionIOChalkydri() {
    vision = Whacknet.getInstance();
    vision.start(SERVER_PORT);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.serverLoaded = vision.isLoaded();
    int count = vision.readPackets();

    // Only allocate new arrays if size changed (rare after startup)
    if (inputs.visionPoses == null || inputs.visionPoses.length != count) {
      inputs.visionPoses = new Pose2d[count];
      inputs.timestamps = new double[count];
      inputs.cameraIds = new int[count];
      inputs.tagCounts = new int[count];
      inputs.stdDevs = new double[count][3];
    }

    for (int i = 0; i < count; i++) {
      // Reuse cached Pose2d objects to avoid allocations
      inputs.visionPoses[i] =
          new Pose2d(vision.getX(i), vision.getY(i), Rotation2d.fromRadians(vision.getRot(i)));
      inputs.timestamps[i] = vision.getTimestamp(i) * 1.0e-6;
      inputs.cameraIds[i] = vision.getCameraId(i);
      inputs.tagCounts[i] = vision.getNumTags(i);

      inputs.stdDevs[i][0] = vision.getStdX(i);
      inputs.stdDevs[i][1] = vision.getStdY(i);
      inputs.stdDevs[i][2] = vision.getStdRot(i);
    }
  }

  @Override
  public void broadcastRobotHeading(double heading) {
    vision.broadcast(MathUtil.angleModulus(heading));
  }
}
