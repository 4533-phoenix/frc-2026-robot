// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

/**
 * Holds enumerated types that define the high-level goals and states of the robot's superstructure.
 *
 * <p>These enums drive the core state machine in {@link Superstructure}, which governs the behavior
 * of the flywheel, hood, indexer, and drive rotation based on driver intent and field conditions.
 */
public class SuperstructureStates {

  /**
   * Represents the current active state of the robot's subsystems as evaluated by the
   * superstructure.
   */
  public enum RobotState {
    /** No active scoring intent. Subsystems are safely stopped or stowed. */
    IDLE,

    /**
     * Preparing to score. Flywheel spins at an idle speed to reduce spin-up latency, but the hood
     * is not yet tracking. Active when the hub is enabled or approaching, but the robot is outside
     * the immediate shooting zone.
     */
    WARMING,

    /**
     * In position to score. Flywheel and hood actively track the target based on distance. The
     * driver retains full manual control over drive rotation.
     */
    TRACKING,

    /**
     * Driver has requested automatic aiming. Flywheel and hood continue tracking, but drive
     * rotation is now overridden to automatically align with the target.
     */
    AIMING,

    /**
     * All aiming conditions are met and the driver is requesting to fire. The indexer actively
     * feeds game pieces into the shooter.
     */
    FIRING,

    /**
     * The robot is in climbing mode. All scoring mechanisms stop and the intake arm retracts to
     * prevent interference or damage.
     */
    CLIMBING
  }

  /** Represents the requested high-level goal from driver input. */
  public enum RobotGoal {
    /** No specific action requested. Default state. */
    IDLE,

    /** Driver wishes to automatically align the robot to score. */
    AIM,

    /** Driver wishes to fire game pieces. Typically also implies AIM. */
    FIRE,

    /** Driver has commanded the climb sequence. Takes priority over scoring goals. */
    CLIMB
  }
}
