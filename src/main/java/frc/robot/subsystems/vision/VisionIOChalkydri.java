// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.VisionNative;
import frc.robot.util.VisionNative.VisionObservation;
import java.util.List;

public class VisionIOChalkydri implements VisionIO {
  public VisionIOChalkydri(int port) {
    VisionNative.start(port);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.serverLoaded = true;
    List<VisionObservation> observations = VisionNative.readPackets();

    int size = observations.size();
    inputs.visionPoses = new Pose2d[size];
    inputs.timestamps = new double[size];
    inputs.cameraIds = new long[size];
    inputs.stdDevs = new double[size][3];

    for (int i = 0; i < size; i++) {
      VisionObservation obs = observations.get(i);
      inputs.visionPoses[i] = obs.getPose();
      inputs.timestamps[i] = obs.getTimestampSeconds();
      inputs.cameraIds[i] = obs.cameraId();

      inputs.stdDevs[i][0] = obs.stdX();
      inputs.stdDevs[i][1] = obs.stdY();
      inputs.stdDevs[i][2] = obs.stdRot();
    }
  }
}
