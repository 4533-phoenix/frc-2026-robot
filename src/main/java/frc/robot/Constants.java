// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
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
  public static final Mode simMode = Mode.SIM;

  /** The currently active operating mode based on hardware detection. */
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** Defines the three possible runtime modes for AdvantageKit. */
  public static enum Mode {
    /** Running on a real robot (roboRIO). */
    REAL,

    /** Running a physics simulator (WPILib sim). */
    SIM,

    /** Replaying from a previously recorded log file. */
    REPLAY
  }

  // Field Dimensions
  /** Total length of the field. */
  public static final Distance fieldLength = Inches.of(651.25);
  /** Total width of the field. */
  public static final Distance fieldWidth = Inches.of(315.5);

  // Field Objects
  /** The location of the center of the field outpost mechanism. */
  public static final Translation2d outpostPosition =
      new Translation2d(Meters.of(4.625), fieldWidth.div(2.0));

  // Game Specific Zones
  /** The area on the field where the robot is permitted to shoot game pieces. */
  public static final Rectangle2d shootingZone =
      new Rectangle2d(Translation2d.kZero, new Translation2d(Meters.of(4.02), fieldWidth));

  /** The area on the field where the robot is permitted to lob game pieces. */
  public static final Rectangle2d lobbingZone =
      new Rectangle2d(
          new Pose2d(fieldLength.div(2.0), fieldWidth.div(2.0), null), Meters.of(6.07), fieldWidth);
}
