// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.util.FieldUtil;

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

  // Field Objects
  /** The location of the center of the field hub mechanism. */
  public static final Translation2d HUB_POSITION =
      new Translation2d(Meters.of(4.6255178), Meters.of(4.0346376));

  /** Center of the left lobbing target line (blue alliance coordinates). */
  public static final Translation2d LOBBING_TARGET_LEFT_CENTER =
      new Translation2d(Meters.of(2.159), Meters.of(5.558));

  /** Center of the right lobbing target line (blue alliance coordinates). */
  public static final Translation2d LOBBING_TARGET_RIGHT_CENTER =
      new Translation2d(Meters.of(2.159), FieldUtil.FIELD_WIDTH.minus(Meters.of(5.558)));

  /** Half-length of each lobbing target line segment (0.5m in each direction from center). */
  public static final Distance LOBBING_TARGET_HALF_LENGTH = Meters.of(0.5);

  // Game Specific Zones
  /** The area on the field where the robot is permitted to shoot game pieces. */
  public static final Rectangle2d SHOOTING_ZONE =
      new Rectangle2d(
          Translation2d.kZero, new Translation2d(Meters.of(4.02), FieldUtil.FIELD_WIDTH));

  /** The area on the field where the robot is permitted to lob game pieces. */
  public static final Rectangle2d LOBBING_ZONE =
      new Rectangle2d(
          new Pose2d(
              FieldUtil.FIELD_LENGTH.div(2.0), FieldUtil.FIELD_WIDTH.div(2.0), Rotation2d.kZero),
          Meters.of(6.07),
          FieldUtil.FIELD_WIDTH);
}
