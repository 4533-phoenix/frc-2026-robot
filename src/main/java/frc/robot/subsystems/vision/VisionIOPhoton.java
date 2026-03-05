// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * PhotonVision implementation of VisionIO for Team 4533. Incorporates 4451's distance-based
 * standard deviation scaling.
 */
public class VisionIOPhoton implements VisionIO {
  private final List<CameraContext> cameras = new ArrayList<>();

  private static class CameraContext {
    public final int id;
    public final PhotonCamera camera;
    public final PhotonPoseEstimator estimator;

    public CameraContext(int id, CameraConfig config) {
      this.id = id;
      this.camera = new PhotonCamera(config.name());
      this.estimator = new PhotonPoseEstimator(fieldLayout, config.robotToCamera());
    }
  }

  public VisionIOPhoton() {
    for (var entry : cameraMap.entrySet()) {
      cameras.add(new CameraContext(entry.getKey(), entry.getValue()));
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    List<Pose2d> poses = new ArrayList<>();
    List<Double> timestamps = new ArrayList<>();
    List<Integer> cameraIds = new ArrayList<>();
    List<Integer> tagCounts = new ArrayList<>();
    List<Double> stdX = new ArrayList<>();
    List<Double> stdY = new ArrayList<>();
    List<Double> stdRot = new ArrayList<>();

    boolean allConnected = true;

    for (CameraContext ctx : cameras) {
      if (!ctx.camera.isConnected()) {
        allConnected = false;
        continue;
      }

      for (PhotonPipelineResult result : ctx.camera.getAllUnreadResults()) {
        if (!result.hasTargets()) continue;

        Optional<EstimatedRobotPose> estimation =
            ctx.estimator
                .estimateCoprocMultiTagPose(result)
                .or(() -> ctx.estimator.estimateLowestAmbiguityPose(result));

        if (estimation.isPresent()) {
          EstimatedRobotPose estimatedPose = estimation.get();
          Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
          List<PhotonTrackedTarget> targets = result.getTargets();

          // Apply single-tag usage filter
          if (targets.size() == 1 && !isUsableSingleTag(targets.get(0))) {
            continue;
          }

          // Calculate Standard Deviations
          Matrix<N3, N1> stdDevs = getEstimationStdDevs(pose2d, targets);

          poses.add(pose2d);
          timestamps.add(estimatedPose.timestampSeconds);
          cameraIds.add(ctx.id);
          tagCounts.add(targets.size());
          stdX.add(stdDevs.get(0, 0));
          stdY.add(stdDevs.get(1, 0));
          stdRot.add(stdDevs.get(2, 0));
        }
      }
    }

    // Update the AutoLogged inputs
    inputs.serverLoaded = allConnected;
    inputs.visionPoses = poses.toArray(new Pose2d[0]);
    inputs.timestamps = timestamps.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.cameraIds = cameraIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.tagCounts = tagCounts.stream().mapToInt(Integer::intValue).toArray();
    inputs.stdDevX = stdX.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.stdDevY = stdY.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.stdDevRot = stdRot.stream().mapToDouble(Double::doubleValue).toArray();
  }

  /** Logic from Riptide's AprilTagAlgorithms.java */
  private Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
    int numTags = 0;
    double totalDistance = 0;

    for (PhotonTrackedTarget target : targets) {
      var tagPose = fieldLayout.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) continue;

      numTags++;
      totalDistance +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    if (numTags == 0) return singleTagStdDevs;

    double avgDistance = totalDistance / numTags;
    Matrix<N3, N1> stdDevs = (numTags > 1) ? multiTagStdDevs : singleTagStdDevs;

    // Increase std devs based on average distance (Riptide formula)
    return stdDevs.times(1 + (avgDistance * avgDistance / 30.0));
  }

  /** Logic from Riptide's SingleTagAlgorithms.java */
  private boolean isUsableSingleTag(PhotonTrackedTarget target) {
    return target.getPoseAmbiguity() < ambiguityCutoff
        && target.getBestCameraToTarget().getTranslation().getNorm() < singleTagPoseCutoffMeters;
  }

  @Override
  public void broadcastRobotHeading(Rotation2d heading) {
    // PhotonPoseEstimator manages heading internally if provided via update(),
    // but we can pass it to the estimator if using constrained modes.
  }
}
