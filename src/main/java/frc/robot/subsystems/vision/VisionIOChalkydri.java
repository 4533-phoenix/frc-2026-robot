// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.VisionNative;
import frc.robot.util.VisionNative.VisionObservation;
import java.util.List;

public class VisionIOChalkydri implements VisionIO {
  private final VisionNative vision;

  public VisionIOChalkydri() {
    vision = VisionNative.getInstance();
    vision.start(SERVER_PORT);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.serverLoaded = vision.isLoaded();
    List<VisionObservation> observations = vision.readPackets();

    // debug print all the observations
    for (VisionObservation obs : observations) {
      System.out.println(
          "Camera ID: "
              + obs.cameraId()
              + ", Tag Count: "
              + obs.numTags()
              + ", Timestamp: "
              + obs.getTimestampSeconds()
              + ", Pose: "
              + obs.getPose()
              + ", StdDevs: ["
              + obs.stdX()
              + ", "
              + obs.stdY()
              + ", "
              + obs.stdRot()
              + "]");
    }

    int size = observations.size();
    inputs.visionPoses = new Pose2d[size];
    inputs.timestamps = new double[size];
    inputs.cameraIds = new long[size];
    inputs.tagCounts = new long[size];
    inputs.stdDevs = new double[size][3];

    for (int i = 0; i < size; i++) {
      VisionObservation obs = observations.get(i);
      inputs.visionPoses[i] = obs.getPose();
      inputs.timestamps[i] = obs.getTimestampSeconds();
      inputs.cameraIds[i] = obs.cameraId();
      inputs.tagCounts[i] = obs.numTags();

      inputs.stdDevs[i][0] = obs.stdX();
      inputs.stdDevs[i][1] = obs.stdY();
      inputs.stdDevs[i][2] = obs.stdRot();
    }
  }

  public void broadcastRobotHeading(double heading) {
    vision.broadcast(heading);
  }
}
