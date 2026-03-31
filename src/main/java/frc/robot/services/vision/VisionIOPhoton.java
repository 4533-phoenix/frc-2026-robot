// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.vision;

import static frc.robot.services.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.IMUState;
import frc.lib.lowlevel.Whacknet;
import frc.robot.services.vision.Vision.VisionObservation;
import frc.robot.services.vision.VisionConstants.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * PhotonVision implementation of VisionIO for Team 4533. Provides raw pose estimations without
 * filtering or dynamic standard deviations.
 */
public class VisionIOPhoton implements VisionIO {
  private final List<CameraContext> cameras = new ArrayList<>();
  private static final Pose2d EMPTY_POSE = new Pose2d();
  private final Whacknet whacknet = BROADCAST_HEADING ? Whacknet.getInstance() : null;

  private final VisionObservation[] observationBuffer = new VisionObservation[2];

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

    // Pre-fill the buffer so it's never null
    observationBuffer[0] = new VisionObservation(EMPTY_POSE, 0.0, 0, 0, 0.0, 0.0, 0.0);
    observationBuffer[1] = new VisionObservation(EMPTY_POSE, 0.0, 1, 0, 0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    for (CameraContext ctx : cameras) {
      if (!ctx.camera.isConnected()) {
        continue;
      }

      List<PhotonPipelineResult> results = ctx.camera.getAllUnreadResults();

      for (PhotonPipelineResult result : results) {
        if (result.hasTargets()) {
          int tagCount = result.getTargets().size();

          Optional<EstimatedRobotPose> normalEstimation =
              ctx.estimator
                  .estimateCoprocMultiTagPose(result)
                  .or(() -> ctx.estimator.estimateLowestAmbiguityPose(result));

          if (normalEstimation.isPresent()) {
            EstimatedRobotPose estimatedPose = normalEstimation.get();
            observationBuffer[0] =
                new VisionObservation(
                    estimatedPose.estimatedPose.toPose2d(),
                    estimatedPose.timestampSeconds,
                    ctx.id,
                    tagCount,
                    0.0,
                    0.0,
                    0.0);
          }
          Optional<EstimatedRobotPose> constrainedEstimation =
              ctx.estimator.estimateCoprocConstrainedPose(result);

          if (constrainedEstimation.isPresent()) {
            EstimatedRobotPose estimatedPose = constrainedEstimation.get();
            observationBuffer[1] =
                new VisionObservation(
                    estimatedPose.estimatedPose.toPose2d(),
                    estimatedPose.timestampSeconds,
                    ctx.id,
                    tagCount,
                    0.0,
                    0.0,
                    0.0);
          }
        }
      }
    }

    inputs.observations = new VisionObservation[] {observationBuffer[0], observationBuffer[1]};
  }

  @SuppressWarnings("unused")
  @Override
  public void broadcastTelemetry(IMUState imuState) {
    if (whacknet != null && imuState != null && BROADCAST_HEADING) {
      whacknet.broadcastTelemetry(
          (long) (imuState.timestampSec() * 1.0e6),
          imuState.rollRad(),
          imuState.pitchRad(),
          imuState.yawRad(),
          imuState.rollVelRadPerSec(),
          imuState.pitchVelRadPerSec(),
          imuState.yawVelRadPerSec(),
          VisionConstants.SERVER_BPORT);
    }
  }
}
