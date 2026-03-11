// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
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
    inputs.serverLoaded = true;
    Pose2d robotPose = poseSupplier.get();
    double timestamp = Timer.getFPGATimestamp();

    int cameraCount = CAMERA_MAP.size();

    // OPTIMIZATION: Resize arrays once. Removed ArrayLists and Java Streams to prevent
    // Autoboxing and continuous memory allocation during simulation.
    if (inputs.visionPoses.length != cameraCount) {
      inputs.visionPoses = new Pose2d[cameraCount];
      inputs.timestamps = new double[cameraCount];
      inputs.cameraIds = new int[cameraCount];
      inputs.tagCounts = new int[cameraCount];

      inputs.stdDevX = new double[cameraCount];
      inputs.stdDevY = new double[cameraCount];
      inputs.stdDevRot = new double[cameraCount];
    }

    int index = 0;

    // Iterate through all configured cameras to simulate their detections
    for (var entry : CAMERA_MAP.entrySet()) {
      int camId = entry.getKey();
      CameraConfig config = entry.getValue();

      // Convert Transform3d to Transform2d for Pose2d transformation
      var t3d = config.robotToCamera();
      var t2d =
          new edu.wpi.first.math.geometry.Transform2d(
              new Translation2d(t3d.getX(), t3d.getY()), new Rotation2d(t3d.getRotation().getZ()));
      Pose2d cameraPose = robotPose.transformBy(t2d);
      int visibleTags = 0;
      double totalDistance = 0.0;

      // Check visibility for each tag on the field
      for (AprilTag tag : FIELD_LAYOUT.getTags()) {
        Pose2d tagPose = tag.pose.toPose2d();
        Translation2d diff = tagPose.getTranslation().minus(cameraPose.getTranslation());
        double distance = diff.getNorm();

        // Check if tag is within range and within camera field of view
        if (distance <= config.maxRangeMeters()) {
          Rotation2d angleToTag = diff.getAngle().minus(cameraPose.getRotation());
          if (Math.abs(angleToTag.getDegrees()) <= config.fovDegrees() / 2.0) {
            visibleTags++;
            totalDistance += distance;
          }
        }
      }

      // Populate parallel arrays
      inputs.cameraIds[index] = camId;
      inputs.timestamps[index] = timestamp;
      inputs.tagCounts[index] = visibleTags;

      if (visibleTags > 0) {
        // Generate dummy pose and standard deviations based on average distance
        double avgDist = totalDistance / visibleTags;

        // Simple noise model where standard deviation increases with distance
        double xyStd = 0.01 * (avgDist * avgDist) / visibleTags;
        double thetaStd = 0.01 * (avgDist * avgDist) / visibleTags;

        inputs.visionPoses[index] = robotPose;
        inputs.stdDevX[index] = xyStd;
        inputs.stdDevY[index] = xyStd;
        inputs.stdDevRot[index] = thetaStd;
      } else {
        // No tags visible send no data
        inputs.visionPoses[index] = EMPTY_POSE;
        inputs.stdDevX[index] = 0.0;
        inputs.stdDevY[index] = 0.0;
        inputs.stdDevRot[index] = 0.0;
      }

      index++;
    }
  }

  @Override
  public void broadcastRobotHeading(Rotation2d heading) {
    // No-op in simulation
  }
}
