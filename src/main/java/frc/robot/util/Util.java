// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
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

  /**
   * Calculates the closest point on a vertical (Y-axis) lobbing line segment to the given position.
   * The line is centered at {@code center} and extends {@code halfLen} meters in each Y direction.
   *
   * @param from The position to calculate the closest point from.
   * @param center The center translation of the vertical line segment.
   * @param halfLen The half length of the vertical line segment in meters.
   * @return The closest point on the line segment to the given position.
   */
  public static Translation2d closestPointOnLobLine(
      Translation2d from, Translation2d center, double halfLen) {
    double clampedY = MathUtil.clamp(from.getY(), center.getY() - halfLen, center.getY() + halfLen);
    return new Translation2d(center.getX(), clampedY);
  }

  /**
   * Calculates the lead offset to apply to the static target, accounting for the robot's current
   * velocity and the estimated time of flight. The lead magnitude is clamped to 50% of the
   * shooter-to-target distance so the virtual target can never overshoot past the real target.
   *
   * @param shooterPos The current position of the shooter.
   * @param targetTranslation The translation of the static target.
   * @param fieldVelocity The field-relative velocity of the robot.
   * @param tofSeconds The estimated time of flight in seconds.
   * @return The calculated and clamped lead translation to apply.
   */
  public static Translation2d calculateClampedLead(
      Translation2d shooterPos,
      Translation2d targetTranslation,
      Translation2d fieldVelocity,
      double tofSeconds) {
    Translation2d rawLead =
        new Translation2d(fieldVelocity.getX() * tofSeconds, fieldVelocity.getY() * tofSeconds);

    double distToTarget = shooterPos.getDistance(targetTranslation);
    double maxLead = distToTarget * 0.5;
    double leadMag = rawLead.getNorm();

    if (leadMag > maxLead && leadMag > 1e-6) {
      return rawLead.times(maxLead / leadMag);
    }
    return rawLead;
  }
}
