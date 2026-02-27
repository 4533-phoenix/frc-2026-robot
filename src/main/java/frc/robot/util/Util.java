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

public class Util {
  public static final double FIELD_LENGTH = 16.5413;

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  public static Rectangle2d flipAllianceIfNeeded(Rectangle2d rectangle) {
    return shouldFlip() ? flipAlliance(rectangle) : rectangle;
  }

  public static Pose2d flipAllianceIfNeeded(Pose2d pose) {
    return shouldFlip() ? flipAlliance(pose) : pose;
  }

  public static Translation2d flipAllianceIfNeeded(Translation2d translation) {
    return shouldFlip() ? flipAlliance(translation) : translation;
  }

  public static Rotation2d flipAllianceIfNeeded(Rotation2d rotation) {
    return shouldFlip() ? flipAlliance(rotation) : rotation;
  }

  public static Rectangle2d flipAlliance(Rectangle2d rectangle) {
    return new Rectangle2d(
        flipAlliance(rectangle.getCenter()), rectangle.getXWidth(), rectangle.getYWidth());
  }

  public static Pose2d flipAlliance(Pose2d pose) {
    return new Pose2d(flipAlliance(pose.getTranslation()), flipAlliance(pose.getRotation()));
  }

  public static Translation2d flipAlliance(Translation2d translation) {
    return new Translation2d(FIELD_LENGTH - translation.getX(), translation.getY());
  }

  public static Rotation2d flipAlliance(Rotation2d rotation) {
    return Rotation2d.fromRadians(Math.PI).minus(rotation);
  }
}
