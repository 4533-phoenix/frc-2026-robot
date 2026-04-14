// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.control.driver.profiles;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.services.control.GenericControlProfile;
import frc.robot.services.control.driver.DriverProfile;

/**
 * A no assists version of the driver profile with straight linear correlation, no deadband, and no
 * rumble.
 */
public class NoAssistsDriverProfile extends GenericControlProfile implements DriverProfile {
  private final LinearVelocity maxLinearVelocity;
  private final AngularVelocity maxAngularVelocity;

  /**
   * Constructs a NoAssistsDriverProfile.
   *
   * @param controller The Xbox controller used for input.
   * @param maxLinearVelocity The maximum linear velocity allowed.
   * @param maxAngularVelocity The maximum angular velocity allowed.
   */
  public NoAssistsDriverProfile(
      XboxController controller,
      LinearVelocity maxLinearVelocity,
      AngularVelocity maxAngularVelocity) {
    super(controller);
    this.maxLinearVelocity = maxLinearVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
  }

  @Override
  public ChassisSpeeds getDesiredSpeeds() {
    return new ChassisSpeeds(
        maxLinearVelocity.times(-controller.getLeftY()),
        maxLinearVelocity.times(-controller.getLeftX()),
        maxAngularVelocity.times(-controller.getRightX()));
  }

  @Override
  public boolean wantsAim() {
    return controller.getLeftTriggerAxis() > 0.5;
  }

  @Override
  public boolean wantsShoot() {
    return controller.getRightTriggerAxis() > 0.5;
  }

  @Override
  public boolean wantsReset() {
    return controller.getStartButton();
  }

  @Override
  public boolean wantsUnjam() {
    return false;
  }
}
