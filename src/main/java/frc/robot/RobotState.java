// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

/**
 * Enumerates the high-level states of the robot's superstructure.
 *
 * <p>The state machine governs flywheel, hood, drive rotation, and indexer behavior. Intake
 * spinners are controlled independently as an orthogonal overlay and are not represented here.
 */
public enum RobotState {
  /** No active scoring intent. Flywheel off, hood stowed. */
  IDLE,

  /**
   * Outpost approaching or enabled but robot is outside the shooting zone. Flywheel at idle spin to
   * reduce spin-up latency. Hood stowed.
   */
  WARMING,

  /**
   * In the shooting zone with outpost enabled. Flywheel and hood actively tracking the target.
   * Driver retains full rotation control.
   */
  TRACKING,

  /**
   * Driver is holding the aim trigger. Drive rotation is PID-controlled toward the target. Flywheel
   * and hood continue tracking.
   */
  AIMING,

  /** Aim conditions met and driver is holding the fire trigger. Indexer feeds game pieces. */
  FIRING,

  /** Driver has initiated a climb. Intake arm retracts, all scoring mechanisms stop. */
  CLIMBING
}
