// Copyright (c) 2025 FRC Team 4451 (Robotz Garage)
// https://github.com/frc4451
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public record VisionSource(String name, Transform3d robotToCamera) {}
