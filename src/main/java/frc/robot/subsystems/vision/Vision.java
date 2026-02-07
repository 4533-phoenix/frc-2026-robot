// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final Drive drive;

  private final Map<Long, Double> lastTimestampMap = new HashMap<>();
  private final Map<Long, Alert> alertMap = new HashMap<>();

  public Vision(VisionIO io, Drive drive) {
    this.io = io;
    this.drive = drive;

    for (var entry : CAMERA_MAP.entrySet()) {
      lastTimestampMap.put(entry.getKey(), Timer.getTimestamp());
      alertMap.put(
          entry.getKey(),
          new Alert("Vision: Camera '" + entry.getValue() + "' is offline!", AlertType.kWarning));
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    io.broadcastRobotHeading(drive.getRotation().getRadians());

    double currentTime = Timer.getTimestamp();

    for (int i = 0; i < inputs.visionPoses.length; i++) {
      long id = inputs.cameraIds[i];
      lastTimestampMap.put(id, currentTime);

      var stds = VecBuilder.fill(inputs.stdDevs[i][0], inputs.stdDevs[i][1], inputs.stdDevs[i][2]);
      drive.addVisionMeasurement(inputs.visionPoses[i], inputs.timestamps[i], stds);
    }

    for (long id : CAMERA_MAP.keySet()) {
      double lastSeen = lastTimestampMap.getOrDefault(id, 0.0);
      boolean isOffline = (currentTime - lastSeen) > OFFLINE_TIMEOUT_SECONDS;

      alertMap.get(id).set(isOffline);
      Logger.recordOutput("Vision/CameraStatus/" + CAMERA_MAP.get(id), !isOffline);
    }
  }
}
