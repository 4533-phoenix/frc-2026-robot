// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Defines constants for the robot's physical dimensions, game field layout, and AdvantageKit
 * operating mode.
 */
public final class Constants {
  /**
   * The mode to use when running in simulation. Change to {@link Mode#REPLAY} to read from a log
   * file instead of simulating physics.
   */
  public static final Mode SIM_MODE = Mode.SIM;

  /** The currently active operating mode based on hardware detection. */
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

  /** Defines the three possible runtime modes for AdvantageKit. */
  public static enum Mode {
    /** Running on a real robot (roboRIO). */
    REAL,

    /** Running a physics simulator (WPILib sim). */
    SIM,

    /** Replaying from a previously recorded log file. */
    REPLAY
  }
}
