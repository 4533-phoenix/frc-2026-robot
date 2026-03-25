// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.monitors;

/**
 * An interface to enforce standard health monitoring and fault management
 * across all robot subsystems.
 */
public interface MonitoredSubsystem {
  /**
   * Returns whether or not the subsystem is healthy.
   *
   * @return True if the subsystem is healthy, false otherwise.
   */
  boolean isHealthy();

  /** Clears all hardware faults and warnings associated with this subsystem. */
  void clearFaults();
}
