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
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Distance fieldLength = Inches.of(651.25);
  public static final Distance fieldWidth = Inches.of(315.5);
  public static final Translation2d outpostPosition =
      new Translation2d(Meters.of(4.625), fieldWidth.div(2.0));
  public static final Rectangle2d shootingZone =
      new Rectangle2d(Translation2d.kZero, new Translation2d(Meters.of(4.02), fieldWidth));
  public static final Rectangle2d lobbingZone =
      new Rectangle2d(
          new Pose2d(fieldLength.div(2.0), fieldWidth.div(2.0), null), Meters.of(6.07), fieldWidth);
}
