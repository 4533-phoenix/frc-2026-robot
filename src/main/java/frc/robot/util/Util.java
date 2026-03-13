// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility class for evaluating game-specific conditions and performing other geometry-related
 * calculations.
 */
public class Util {
  private Util() {} // Prevent instantiation

  private static final String MATCH_MODE_KEY = "Match Mode";

  /**
   * Evaluates the game specific message to determine if the hub is enabled at a given match time.
   *
   * @param time The match time in seconds (counts down from 135 to 0).
   * @return True if the hub is enabled, false otherwise.
   */
  public static boolean isHubEnabledAtTime(double time) {
    boolean isTimedMatch = DriverStation.isFMSAttached() || time > 0;

    if (!isTimedMatch) {
      return DriverStation.isEnabled();
    }

    if (DriverStation.isAutonomous() || time > 130 || time <= 30) return true;

    String data = DriverStation.getGameSpecificMessage();
    var alliance = DriverStation.getAlliance();
    if (data == null || data.isEmpty() || alliance.isEmpty()) return true;

    char inactiveInShift1 = data.charAt(0);
    char myColor = (alliance.get() == Alliance.Red) ? 'R' : 'B';
    boolean amIInactiveInShift1 = (inactiveInShift1 == myColor);

    if (time > 105) return !amIInactiveInShift1;
    else if (time > 80) return amIInactiveInShift1;
    else if (time > 55) return !amIInactiveInShift1;
    else return amIInactiveInShift1;
  }

  /**
   * Evaluates if the hub is currently active.
   *
   * @return True if the hub is currently active.
   */
  public static boolean isHubEnabled() {
    return isHubEnabledAtTime(DriverStation.getMatchTime());
  }

  /**
   * Evaluates if the hub will enable within the next 5 seconds.
   *
   * @return True if the hub will enable within the next 5 seconds.
   */
  public static boolean isHubApproaching() {
    double time = DriverStation.getMatchTime();
    double futureTime = time - 5.0;

    if (futureTime <= 0) return false;

    boolean futureEnabled = isHubEnabledAtTime(futureTime);
    return futureEnabled && !isHubEnabled();
  }

  /**
   * Checks if match mode is active. Match mode enables automatically when a match timer is running
   * (FMS or driver station practice mode). It can also be forced on via the dashboard toggle for
   * testing.
   *
   * @return True when match mode is active.
   */
  public static boolean isMatchMode() {
    if (SmartDashboard.getBoolean(MATCH_MODE_KEY, false)) return true;
    double time = DriverStation.getMatchTime();
    return DriverStation.isFMSAttached() || time > 0;
  }

  /**
   * Checks if match mode is active only because of the SmartDashboard override toggle, not because
   * a real FMS/timed match is running. Used to allow actions (e.g. climbing) that are normally
   * gated behind endgame when testing without a real match.
   *
   * @return True when match mode is active only because of the dashboard override.
   */
  public static boolean isMatchModeOverridden() {
    boolean dashboardOverride = SmartDashboard.getBoolean(MATCH_MODE_KEY, false);
    double time = DriverStation.getMatchTime();
    boolean realMatch = DriverStation.isFMSAttached() || time > 0;
    return dashboardOverride && !realMatch;
  }

  /**
   * Checks if the match is in the last 30 seconds of teleop.
   *
   * @return True when the match is in the last 30 seconds of teleop.
   */
  public static boolean isEndgame() {
    if (DriverStation.isAutonomous()) return false;
    double time = DriverStation.getMatchTime();
    boolean isTimedMatch = DriverStation.isFMSAttached() || time > 0;
    return isTimedMatch && time <= 30.0 && time > 0;
  }

  /** Publish the match mode state to the SmartDashboard for testing purposes. */
  public static void publishMatchMode() {
    SmartDashboard.putBoolean(MATCH_MODE_KEY, isMatchMode());
  }
}
