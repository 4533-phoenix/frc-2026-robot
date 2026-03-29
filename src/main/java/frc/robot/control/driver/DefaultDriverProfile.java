// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.control.driver;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.control.GenericControlProfile;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.BooleanSupplier;

public class DefaultDriverProfile extends GenericControlProfile implements DriverProfile {
  private final LinearVelocity maxLinearVelocity;
  private final AngularVelocity maxAngularVelocity;
  private final BooleanSupplier isRobotReadyToFire;

  public DefaultDriverProfile(
      CommandXboxController controller,
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
        maxLinearVelocity.times(processJoystick(controller.getLeftY())),
        maxLinearVelocity.times(processJoystick(controller.getLeftX())),
        maxAngularVelocity.times(processJoystick(controller.getRightX())));
  }

  @Override
  public boolean wantsAim() {
    return controller.leftTrigger().getAsBoolean();
  }

  @Override
  public boolean wantsShoot() {
    return controller.rightTrigger().getAsBoolean();
  }

  @Override
  public boolean wantsReset() {
    return controller.start().getAsBoolean();
  }
}
