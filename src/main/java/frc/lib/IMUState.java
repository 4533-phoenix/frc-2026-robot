// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

/** Represents a 6-DOF high-frequency snapshot for vision correction. */
public record IMUState(
    double timestampSec,
    double rollRad,
    double pitchRad,
    double yawRad,
    double rollVelRadPerSec,
    double pitchVelRadPerSec,
    double yawVelRadPerSec) {}
