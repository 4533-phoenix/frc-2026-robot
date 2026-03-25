// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.Vision.VisionObservation;
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
  private static final Pose2d EMPTY_POSE = new Pose2d();

  private static final int MAX_OBSERVATIONS = 64;
  private final VisionObservation[] observationBuffer = new VisionObservation[MAX_OBSERVATIONS];

  private static class CameraContext {
    public final int id;
    public final PhotonCamera camera;
    public final PhotonPoseEstimator estimator;

    public CameraContext(int id, CameraConfig config) {
      this.id = id;
      this.camera = new PhotonCamera(config.name());
      this.estimator = new PhotonPoseEstimator(FIELD_LAYOUT, config.robotToCamera());
    }
  }

  /** Creates a new VisionIOPhoton and connects to cameras. */
  public VisionIOPhoton() {
    for (var entry : CAMERA_MAP.entrySet()) {
      cameras.add(new CameraContext(entry.getKey(), entry.getValue()));
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    int count = 0;
    double now = Timer.getFPGATimestamp();

    for (CameraContext ctx : cameras) {
      if (!ctx.camera.isConnected()) {
        continue;
      }

      List<PhotonPipelineResult> results = ctx.camera.getAllUnreadResults();

      // If no new frames arrived this loop, send a heartbeat observation at current time
      if (results.isEmpty() && count < MAX_OBSERVATIONS) {
        observationBuffer[count++] = new VisionObservation(EMPTY_POSE, now, ctx.id, 0, 0, 0, 0);
      } else {
        for (PhotonPipelineResult result : results) {
          if (count >= MAX_OBSERVATIONS) break;

          if (result.hasTargets()) {
            Optional<EstimatedRobotPose> estimation =
                ctx.estimator
                    .estimateCoprocMultiTagPose(result)
                    .or(() -> ctx.estimator.estimateLowestAmbiguityPose(result));

            if (estimation.isPresent()) {
              EstimatedRobotPose estimatedPose = estimation.get();
              Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
              List<PhotonTrackedTarget> targets = result.getTargets();

              if (targets.size() == 1 && !isUsableSingleTag(targets.get(0))) {
                // Log frame metadata but discard pose for single tag ambiguity
                observationBuffer[count++] =
                    new VisionObservation(
                        EMPTY_POSE, result.getTimestampSeconds(), ctx.id, 0, 0, 0, 0);
                continue;
              }

              Matrix<N3, N1> stdDevs = getEstimationStdDevs(pose2d, targets);
              observationBuffer[count++] =
                  new VisionObservation(
                      pose2d,
                      estimatedPose.timestampSeconds,
                      ctx.id,
                      targets.size(),
                      stdDevs.get(0, 0),
                      stdDevs.get(1, 0),
                      stdDevs.get(2, 0));
            } else {
              // Camera saw targets but couldn't solve pose; log as 0-tag heartbeat
              observationBuffer[count++] =
                  new VisionObservation(
                      EMPTY_POSE, result.getTimestampSeconds(), ctx.id, 0, 0, 0, 0);
            }
          } else {
            // Camera is connected and frame received, but no targets visible
            observationBuffer[count++] =
                new VisionObservation(EMPTY_POSE, result.getTimestampSeconds(), ctx.id, 0, 0, 0, 0);
          }
        }
      }
    }

    inputs.observations = Arrays.copyOf(observationBuffer, count);
  }

  private Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
    int numTags = 0;
    double totalDistance = 0;

    for (PhotonTrackedTarget target : targets) {
      var tagPose = FIELD_LAYOUT.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      totalDistance +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    if (numTags == 0) return SINGLE_TAG_STD_DEVS;
    double avgDistance = totalDistance / numTags;
    Matrix<N3, N1> stdDevs = (numTags > 1) ? MULTI_TAG_STD_DEVS : SINGLE_TAG_STD_DEVS;
    return stdDevs.times(1 + (avgDistance * avgDistance / 30.0));
  }

  private boolean isUsableSingleTag(PhotonTrackedTarget target) {
    return target.getPoseAmbiguity() < AMBIGUITY_CUTOFF
        && target.getBestCameraToTarget().getTranslation().getNorm()
            < SINGLE_TAG_POSE_CUTOFF.in(Meters);
  }
}
