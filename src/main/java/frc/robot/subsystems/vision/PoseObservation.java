// Copyright (c) 2025 FRC Team 4451 (Robotz Garage)
// https://github.com/frc4451
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionConstants.PoseEstimationMethod;

public record PoseObservation(
    Pose3d robotPose,
    double timestampSeconds,
    double ambiguity,
    int id,
    Matrix<N3, N1> stdDevs,
    PoseEstimationMethod method) {}
