// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Simulation implementation of {@link VisionIO}.
 *
 * <p>This class simulates camera detection of AprilTags based on the robot's current pose
 * and the configured camera FOV and range limits. It calculates dummy pose estimates and
 * standard deviations for simulation testing.
 */
public class VisionIOSim implements VisionIO {
  /** Supplier to get the current robot pose from the physics simulator. */
  private final Supplier<Pose2d> poseSupplier;
  /** Layout of AprilTags on the field. */
  private final AprilTagFieldLayout layout;

  /**
   * Creates a new VisionIOSim.
   *
   * @param poseSupplier A supplier returning the current simulated robot pose.
   */
  public VisionIOSim(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    // Load default field layout (change to specific 2026 field layout when available)
    this.layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
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

    List<Pose2d> poses = new ArrayList<>();
    List<Double> timestamps = new ArrayList<>();
    List<double[]> stdDevs = new ArrayList<>();
    List<Integer> ids = new ArrayList<>();
    List<Integer> tags = new ArrayList<>();

    // Iterate through all configured cameras to simulate their detections
    for (var entry : VisionConstants.cameraMap.entrySet()) {
      int camId = entry.getKey();
      CameraConfig config = entry.getValue();

      // Calculate camera position on the field
      Pose2d cameraPose = robotPose.transformBy(config.robotToCamera());
      int visibleTags = 0;
      double totalDistance = 0.0;

      // Check visibility for each tag on the field
      for (AprilTag tag : layout.getTags()) {
        Pose2d tagPose = tag.pose.toPose2d();
        Translation2d diff = tagPose.getTranslation().minus(cameraPose.getTranslation());
        double distance = diff.getNorm();

        // Check if tag is within range and within camera field of view (FOV)
        if (distance <= config.maxRangeMeters()) {
          Rotation2d angleToTag = diff.getAngle().minus(cameraPose.getRotation());
          if (Math.abs(angleToTag.getDegrees()) <= config.fovDegrees() / 2.0) {
            visibleTags++;
            totalDistance += distance;
          }
        }
      }

      // ALWAYS add an entry for the heartbeat, even if visibleTags is 0
      ids.add(camId);
      timestamps.add(timestamp);
      tags.add(visibleTags);

      if (visibleTags > 0) {
        // Generate dummy pose and standard deviations based on average distance
        double avgDist = totalDistance / visibleTags;
        // Simple noise model: standard deviation increases with distance
        double xyStd = 0.01 * (avgDist * avgDist) / visibleTags;
        double thetaStd = 0.01 * (avgDist * avgDist) / visibleTags;

        poses.add(robotPose); // In real life this would be a noisy measurement
        stdDevs.add(new double[] {xyStd, xyStd, thetaStd});
      } else {
        // No tags visible, send dummy data
        poses.add(new Pose2d());
        stdDevs.add(new double[] {0.0, 0.0, 0.0});
      }
    }

    // Convert lists to arrays for AdvantageKit logging
    inputs.visionPoses = poses.toArray(new Pose2d[0]);
    inputs.timestamps = timestamps.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.cameraIds = ids.stream().mapToInt(Integer::intValue).toArray();
    inputs.tagCounts = tags.stream().mapToInt(Integer::intValue).toArray();
    inputs.stdDevs = stdDevs.toArray(new double[0][0]);
  }

  @Override
  public void broadcastRobotHeading(double heading) {
    // No-op in simulation
  }
}
