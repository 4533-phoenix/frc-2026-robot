// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.vision;

import static frc.robot.services.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.services.vision.VisionConstants.CameraConfig;
import java.util.function.Supplier;

/**
 * Simulation implementation of {@link VisionIO}.
 *
 * <p>This class simulates camera detection of AprilTags based on the robot's current pose and the
 * configured camera FOV and range limits. It uses a 3D perspective projection model to determine
 * exact corner visibility and orientation requirements, matching the behavior of sqpnp solvers.
 */
public class VisionIOSim implements VisionIO {
  private final Supplier<Pose2d> poseSupplier;

  // Pre-allocated empty pose to avoid creating objects constantly when no tags are seen
  private static final Pose3d EMPTY_POSE = new Pose3d();
  private static final Rotation3d EMPTY_ROTATION = new Rotation3d();

  private static final double TAG_SIZE = 0.1651;
  private static final double CORNER_DIST = TAG_SIZE / 2.0;

  // Pre-allocated corners in the tag's local coordinate system (NWU frame)
  // X is outward normal, Y is left, Z is up
  private static final Translation3d[] TAG_CORNERS_LOCAL = {
    new Translation3d(0, CORNER_DIST, -CORNER_DIST),
    new Translation3d(0, -CORNER_DIST, -CORNER_DIST),
    new Translation3d(0, -CORNER_DIST, CORNER_DIST),
    new Translation3d(0, CORNER_DIST, CORNER_DIST)
  };

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
    Pose2d robotPose2d = poseSupplier.get();
    Pose3d robotPose = new Pose3d(robotPose2d);
    double timestamp = Timer.getFPGATimestamp();

    int cameraCount = CAMERA_MAP.size();
    if (inputs.visionPoses.length != cameraCount) {
      inputs.visionPoses = new Pose3d[cameraCount];
      inputs.timestamps = new double[cameraCount];
      inputs.cameraIds = new int[cameraCount];
      inputs.tagCounts = new int[cameraCount];
      inputs.stdDevXs = new double[cameraCount];
      inputs.stdDevYs = new double[cameraCount];
      inputs.stdDevRots = new double[cameraCount];
    }

    int index = 0;
    for (var entry : CAMERA_MAP.entrySet()) {
      int camId = entry.getKey();
      CameraConfig config = entry.getValue();

      // Transform robot pose to camera pose
      Pose3d cameraPose = robotPose.transformBy(config.robotToCamera());
      Translation3d camPos = cameraPose.getTranslation();

      // Calculate FOV limits (Assuming a standard 16:9 Aspect Ratio)
      double hFovTan = Math.tan(Math.toRadians(config.fovDegrees() / 2.0));
      double vFovTan = hFovTan * (9.0 / 16.0);
      double maxRange = config.maxRangeMeters();

      int visibleTags = 0;
      double totalDistance = 0.0;

      for (AprilTag tag : FIELD_LAYOUT.getTags()) {
        Pose3d tagPose = tag.pose;
        Translation3d tagPos = tagPose.getTranslation();

        // Distance vector from Tag to Camera
        Translation3d toCam = camPos.minus(tagPos);
        double dist = toCam.getNorm();

        if (dist < 0.1 || dist > maxRange) {
          continue;
        }

        // Tag Normal (In WPILib NWU, normal points out of +X)
        Translation3d tagNormal = new Translation3d(1.0, 0.0, 0.0).rotateBy(tagPose.getRotation());
        Translation3d dirToCam = toCam.div(dist);

        // Check if the tag is physically facing the camera (Dot product > 0.25 is ~75 degree
        // cutoff)
        double viewingAngleDot =
            tagNormal.getX() * dirToCam.getX()
                + tagNormal.getY() * dirToCam.getY()
                + tagNormal.getZ() * dirToCam.getZ();

        if (viewingAngleDot < 0.25) {
          continue;
        }

        // Perspective Projection Check for all 4 corners
        boolean valid = true;
        for (Translation3d cornerLocal : TAG_CORNERS_LOCAL) {
          // Transform local corner to world coordinate
          Translation3d cornerWorld = tagPos.plus(cornerLocal.rotateBy(tagPose.getRotation()));

          // Translate world coordinate into the local camera frame
          Translation3d cornerCam =
              new Pose3d(cornerWorld, EMPTY_ROTATION).relativeTo(cameraPose).getTranslation();

          // Camera frame: X is forward depth, Y is left/right, Z is up/down
          if (cornerCam.getX() < 0.1 || cornerCam.getX() > maxRange) {
            valid = false;
            break; // Behind lens or out of range
          }

          // Project to 2D
          double u = Math.abs(cornerCam.getY() / cornerCam.getX());
          double v = Math.abs(cornerCam.getZ() / cornerCam.getX());

          // Check if projection falls within FOV limits
          if (u > hFovTan || v > vFovTan) {
            valid = false;
            break; // Corner is off-screen
          }
        }

        if (valid) {
          visibleTags++;
          totalDistance += dist;
        }
      }

      double xyStd = 0.0, thetaStd = 0.0;
      if (visibleTags > 0) {
        double avgDist = totalDistance / visibleTags;
        // Simple noise scaling model depending on distance & amount of tags
        xyStd = 0.01 * (avgDist * avgDist) / visibleTags;
        thetaStd = 0.01 * (avgDist * avgDist) / visibleTags;
      }

      inputs.visionPoses[index] = visibleTags > 0 ? new Pose3d(robotPose2d) : EMPTY_POSE;
      inputs.timestamps[index] = timestamp;
      inputs.cameraIds[index] = camId;
      inputs.tagCounts[index] = visibleTags;
      inputs.stdDevXs[index] = xyStd;
      inputs.stdDevYs[index] = xyStd;
      inputs.stdDevRots[index] = thetaStd;
      index++;
    }
  }
}
