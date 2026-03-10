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
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.ArrayList;
import java.util.Arrays;
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

  // Pre-allocated buffers to avoid ArrayList/stream/boxing allocations every cycle.
  // Sized to a reasonable max; if more results arrive they are silently dropped.
  private static final int MAX_OBSERVATIONS = 16;
  private final Pose2d[] poseBuffer = new Pose2d[MAX_OBSERVATIONS];
  private final double[] timestampBuffer = new double[MAX_OBSERVATIONS];
  private final int[] cameraIdBuffer = new int[MAX_OBSERVATIONS];
  private final int[] tagCountBuffer = new int[MAX_OBSERVATIONS];
  private final double[] stdXBuffer = new double[MAX_OBSERVATIONS];
  private final double[] stdYBuffer = new double[MAX_OBSERVATIONS];
  private final double[] stdRotBuffer = new double[MAX_OBSERVATIONS];

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

  /** Creates a new VisionIOPhoton and connects to cameras. */
  public VisionIOPhoton() {
    for (var entry : cameraMap.entrySet()) {
      cameras.add(new CameraContext(entry.getKey(), entry.getValue()));
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    int count = 0;
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

        if (estimation.isPresent() && count < MAX_OBSERVATIONS) {
          EstimatedRobotPose estimatedPose = estimation.get();
          Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
          List<PhotonTrackedTarget> targets = result.getTargets();

          // Apply single-tag usage filter
          if (targets.size() == 1 && !isUsableSingleTag(targets.get(0))) {
            continue;
          }

          // Calculate Standard Deviations
          Matrix<N3, N1> stdDevs = getEstimationStdDevs(pose2d, targets);

          poseBuffer[count] = pose2d;
          timestampBuffer[count] = estimatedPose.timestampSeconds;
          cameraIdBuffer[count] = ctx.id;
          tagCountBuffer[count] = targets.size();
          stdXBuffer[count] = stdDevs.get(0, 0);
          stdYBuffer[count] = stdDevs.get(1, 0);
          stdRotBuffer[count] = stdDevs.get(2, 0);
          count++;
        }
      }
    }

    // Copy from buffers into exact-sized arrays for AutoLog
    inputs.serverLoaded = allConnected;
    inputs.visionPoses = Arrays.copyOf(poseBuffer, count);
    inputs.timestamps = Arrays.copyOf(timestampBuffer, count);
    inputs.cameraIds = Arrays.copyOf(cameraIdBuffer, count);
    inputs.tagCounts = Arrays.copyOf(tagCountBuffer, count);
    inputs.stdDevX = Arrays.copyOf(stdXBuffer, count);
    inputs.stdDevY = Arrays.copyOf(stdYBuffer, count);
    inputs.stdDevRot = Arrays.copyOf(stdRotBuffer, count);
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
