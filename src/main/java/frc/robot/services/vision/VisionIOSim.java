// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.vision;

import static frc.robot.services.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.services.vision.Vision.VisionObservation;
import frc.robot.services.vision.VisionConstants.CameraConfig;
import java.util.function.Supplier;

/**
 * Simulation implementation of {@link VisionIO}.
 *
 * <p>This class simulates camera detection of AprilTags based on the robot's current pose and the
 * configured camera FOV and range limits. It calculates dummy pose estimates and standard
 * deviations for simulation testing, utilizing zero-allocation arrays to maintain performance.
 */
public class VisionIOSim implements VisionIO {
  private final Supplier<Pose2d> poseSupplier;

  // Pre-allocated empty pose to avoid creating 'new Pose2d()' constantly when no tags are seen
  private static final Pose2d EMPTY_POSE = new Pose2d();

  /**
   * Creates a new VisionIOSim.
   *
   * @param poseSupplier A supplier returning the current simulated robot pose.
   */
  public VisionIOSim(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  /**
   * Updates inputs by simulating camera detections based on robot pose.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Pose2d robotPose = poseSupplier.get();
    double timestamp = Timer.getFPGATimestamp();

    int cameraCount = CAMERA_MAP.size();
    if (inputs.observations.length != cameraCount) {
      inputs.observations = new VisionObservation[cameraCount];
    }

    int index = 0;
    for (var entry : CAMERA_MAP.entrySet()) {
      int camId = entry.getKey();
      CameraConfig config = entry.getValue();

      var t3d = config.robotToCamera();
      var t2d =
          new edu.wpi.first.math.geometry.Transform2d(
              new Translation2d(t3d.getX(), t3d.getY()), new Rotation2d(t3d.getRotation().getZ()));
      Pose2d cameraPose = robotPose.transformBy(t2d);
      int visibleTags = 0;
      double totalDistance = 0.0;

      for (AprilTag tag : FIELD_LAYOUT.getTags()) {
        Pose2d tagPose = tag.pose.toPose2d();
        Translation2d diff = tagPose.getTranslation().minus(cameraPose.getTranslation());
        double distance = diff.getNorm();

        if (distance <= config.maxRangeMeters()) {
          Rotation2d angleToTag = diff.getAngle().minus(cameraPose.getRotation());
          if (Math.abs(angleToTag.getDegrees()) <= config.fovDegrees() / 2.0) {
            visibleTags++;
            totalDistance += distance;
          }
        }
      }

      double xyStd = 0.0, thetaStd = 0.0;
      if (visibleTags > 0) {
        double avgDist = totalDistance / visibleTags;
        xyStd = 0.01 * (avgDist * avgDist) / visibleTags;
        thetaStd = 0.01 * (avgDist * avgDist) / visibleTags;
      }

      inputs.observations[index++] =
          new VisionObservation(
              visibleTags > 0 ? robotPose : EMPTY_POSE,
              timestamp,
              camId,
              visibleTags,
              xyStd,
              xyStd,
              thetaStd);
    }
  }
}
