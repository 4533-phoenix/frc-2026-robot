// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Utility class for common robot math and field geometry operations.
 *
 * <p>Includes functions to automatically flip field coordinates based on the current alliance
 * color.
 */
public class Util {
  /** The length of the FRC field in meters. */
  public static final double FIELD_LENGTH = 16.5413;

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
    return new Translation2d(FIELD_LENGTH - translation.getX(), translation.getY());
  }

  /**
   * Forces a flip of a {@link Rotation2d} to the opposite side of the field.
   *
   * @param rotation The rotation to flip.
   * @return The flipped rotation.
   */
  public static Rotation2d flipAlliance(Rotation2d rotation) {
    return Rotation2d.fromRadians(Math.PI).minus(rotation);
  }
}
