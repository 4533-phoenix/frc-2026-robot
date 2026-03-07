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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Utility class for common robot math and field geometry operations.
 *
 * <p>Includes functions to automatically flip field coordinates based on the current alliance
 * color.
 */
public class Util {
  private static final String MATCH_MODE_KEY = "Match Mode";

  /**
   * Checks if the robot needs to flip its coordinate system for the Red alliance.
   *
   * @return True if on the Red alliance, false if Blue or unknown.
   */
  public static boolean shouldFlip() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /**
   * Flips a {@link Rectangle2d} to the opposite side of the field if on the Red alliance.
   *
   * @param rectangle The rectangle to flip.
   * @return The flipped rectangle, or the original if not needed.
   */
  public static Rectangle2d flipAllianceIfNeeded(Rectangle2d rectangle) {
    return shouldFlip() ? flipAlliance(rectangle) : rectangle;
  }

  /**
   * Flips a {@link Pose2d} to the opposite side of the field if on the Red alliance.
   *
   * @param pose The pose to flip.
   * @return The flipped pose, or the original if not needed.
   */
  public static Pose2d flipAllianceIfNeeded(Pose2d pose) {
    return shouldFlip() ? flipAlliance(pose) : pose;
  }

  /**
   * Flips a {@link Translation2d} to the opposite side of the field if on the Red alliance.
   *
   * @param translation The translation to flip.
   * @return The flipped translation, or the original if not needed.
   */
  public static Translation2d flipAllianceIfNeeded(Translation2d translation) {
    return shouldFlip() ? flipAlliance(translation) : translation;
  }

  /**
   * Flips a {@link Rotation2d} to the opposite side of the field if on the Red alliance.
   *
   * @param rotation The rotation to flip.
   * @return The flipped rotation, or the original if not needed.
   */
  public static Rotation2d flipAllianceIfNeeded(Rotation2d rotation) {
    return shouldFlip() ? flipAlliance(rotation) : rotation;
  }

  /**
   * Forces a flip of a {@link Rectangle2d} to the opposite side of the field.
   *
   * @param rectangle The rectangle to flip.
   * @return The flipped rectangle.
   */
  public static Rectangle2d flipAlliance(Rectangle2d rectangle) {
    return new Rectangle2d(
        flipAlliance(rectangle.getCenter()), rectangle.getXWidth(), rectangle.getYWidth());
  }

  /**
   * Forces a flip of a {@link Pose2d} to the opposite side of the field.
   *
   * @param pose The pose to flip.
   * @return The flipped pose.
   */
  public static Pose2d flipAlliance(Pose2d pose) {
    return new Pose2d(flipAlliance(pose.getTranslation()), flipAlliance(pose.getRotation()));
  }

  /**
   * Forces a flip of a {@link Translation2d} to the opposite side of the field.
   *
   * @param translation The translation to flip.
   * @return The flipped translation.
   */
  public static Translation2d flipAlliance(Translation2d translation) {
    return new Translation2d(
        Constants.fieldLength.minus(translation.getMeasureX()),
        Constants.fieldWidth.minus(translation.getMeasureY()));
  }

  /**
   * Forces a flip of a {@link Rotation2d} to the opposite side of the field.
   *
   * @param rotation The rotation to flip.
   * @return The flipped rotation.
   */
  public static Rotation2d flipAlliance(Rotation2d rotation) {
    return rotation.plus(Rotation2d.kPi);
  }

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

    if (DriverStation.isDisabled()) return false;
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

  /** Returns true if the hub is currently active. */
  public static boolean isHubEnabled() {
    return isHubEnabledAtTime(DriverStation.getMatchTime());
  }

  /** Returns true if the hub will enable within the next 5 seconds. */
  public static boolean isHubApproaching() {
    double time = DriverStation.getMatchTime();
    double futureTime = time - 5.0;

    if (futureTime <= 0) return false;

    boolean futureEnabled = isHubEnabledAtTime(futureTime);
    return futureEnabled && !isHubEnabled();
  }

  /**
   * Returns true when match mode is active. Match mode enables automatically when a match timer is
   * running (FMS or driver station practice mode). It can also be forced on via the dashboard
   * toggle for testing.
   */
  public static boolean isMatchMode() {
    if (SmartDashboard.getBoolean(MATCH_MODE_KEY, false)) return true;
    double time = DriverStation.getMatchTime();
    return DriverStation.isFMSAttached() || time > 0;
  }

  /**
   * Returns true when match mode is active only because of the SmartDashboard override toggle, not
   * because a real FMS/timed match is running. Used to allow actions (e.g. climbing) that are
   * normally gated behind endgame when testing without a real match.
   */
  public static boolean isMatchModeOverridden() {
    boolean dashboardOverride = SmartDashboard.getBoolean(MATCH_MODE_KEY, false);
    double time = DriverStation.getMatchTime();
    boolean realMatch = DriverStation.isFMSAttached() || time > 0;
    return dashboardOverride && !realMatch;
  }

  /** Returns true when the match is in the last 30 seconds of teleop. */
  public static boolean isEndgame() {
    if (DriverStation.isDisabled() || DriverStation.isAutonomous()) return false;
    double time = DriverStation.getMatchTime();
    boolean isTimedMatch = DriverStation.isFMSAttached() || time > 0;
    return isTimedMatch && time <= 30.0 && time > 0;
  }

  /**
   * Returns the closest point on a vertical (Y-axis) lobbing line segment to the given position.
   * The line is centered at {@code center} and extends {@code halfLen} meters in each Y direction.
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
