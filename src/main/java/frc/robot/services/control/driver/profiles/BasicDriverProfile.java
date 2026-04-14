// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.services.control.driver.profiles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.services.control.GenericControlProfile;
import frc.robot.services.control.driver.DriverProfile;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.BooleanSupplier;

/** Default implementation of the DriverProfile for controlling the robot drivetrain. */
public class BasicDriverProfile extends GenericControlProfile implements DriverProfile {
  private final LinearVelocity maxLinearVelocity;
  private final AngularVelocity maxAngularVelocity;
  private final BooleanSupplier isRobotReadyToFire;

  /**
   * Constructs a DefaultDriverProfile.
   *
   * @param controller The Xbox controller used for input.
   * @param maxLinearVelocity The maximum linear velocity allowed.
   * @param maxAngularVelocity The maximum angular velocity allowed.
   * @param isRobotReadyToFire BooleanSupplier indicating if the robot is ready to fire.
   */
  public BasicDriverProfile(
      XboxController controller,
      LinearVelocity maxLinearVelocity,
      AngularVelocity maxAngularVelocity,
      BooleanSupplier isRobotReadyToFire) {
    super(controller);
    this.maxLinearVelocity = maxLinearVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
    this.isRobotReadyToFire = isRobotReadyToFire;
  }

  @Override
  public double getLeftRumble() {
    if (wantsAim() && isRobotReadyToFire.getAsBoolean()) {
      return 0.5;
    }
    return 0;
  }

  @Override
  public double getRightRumble() {
    if (wantsAim() && isRobotReadyToFire.getAsBoolean()) {
      return 0.5;
    }
    return 0;
  }

  /** Centralized cubing and deadband logic */
  private double processJoystick(double input) {
    double deadbanded = MathUtil.applyDeadband(input, DriveConstants.JOYSTICK_DEADBAND);
    return Math.copySign(deadbanded * deadbanded * deadbanded, deadbanded);
  }

  @Override
  public ChassisSpeeds getDesiredSpeeds() {
    return new ChassisSpeeds(
        maxLinearVelocity.times(processJoystick(-controller.getLeftY())),
        maxLinearVelocity.times(processJoystick(-controller.getLeftX())),
        maxAngularVelocity.times(processJoystick(-controller.getRightX())));
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
