// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.control.driver;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.control.GenericControlProfile;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A driver profile that generates smooth, kinematically limited setpoints. Limits translational
 * acceleration as a 2D vector to prevent swerve module skewing, and limits rotational acceleration
 * to prevent wheel slip. This is inspired by 254's approach.
 */
public class SetpointDriverProfile extends GenericControlProfile implements DriverProfile {
  private final LinearVelocity maxLinearVelocity;
  private final LinearAcceleration maxLinearAcceleration;
  private final AngularVelocity maxAngularVelocity;
  private final AngularAcceleration maxAngularAcceleration;
  private final Supplier<ChassisSpeeds> currentSpeedsSupplier;
  private final BooleanSupplier isRobotReadyToFire;

  private ChassisSpeeds lastSetpoint = new ChassisSpeeds();
  private double lastTime = Timer.getFPGATimestamp();

  /**
   * Constructs a SetpointDriverProfile.
   *
   * @param controller The Xbox controller used for input.
   * @param maxLinearVelocity The maximum linear velocity allowed.
   * @param maxLinearAcceleration The maximum linear acceleration allowed.
   * @param maxAngularVelocity The maximum angular velocity allowed.
   * @param maxAngularAcceleration The maximum angular acceleration allowed.
   * @param currentSpeedsSupplier Supplier for the robot's actual measured speeds.
   * @param isRobotReadyToFire BooleanSupplier indicating if the robot is ready to fire.
   */
  public SetpointDriverProfile(
      CommandXboxController controller,
      LinearVelocity maxLinearVelocity,
      LinearAcceleration maxLinearAcceleration,
      AngularVelocity maxAngularVelocity,
      AngularAcceleration maxAngularAcceleration,
      Supplier<ChassisSpeeds> currentSpeedsSupplier,
      BooleanSupplier isRobotReadyToFire) {
    super(controller);
    this.maxLinearVelocity = maxLinearVelocity;
    this.maxLinearAcceleration = maxLinearAcceleration;
    this.maxAngularVelocity = maxAngularVelocity;
    this.maxAngularAcceleration = maxAngularAcceleration;
    this.currentSpeedsSupplier = currentSpeedsSupplier;
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

  private double processJoystick(double input) {
    double deadbanded = MathUtil.applyDeadband(input, DriveConstants.JOYSTICK_DEADBAND);
    return Math.copySign(deadbanded * deadbanded * deadbanded, deadbanded);
  }

  @Override
  public ChassisSpeeds getDesiredSpeeds() {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    // If the loop took way too long (e.g. robot was disabled, or breakpoint hit)
    // we sync the internal setpoint to the robot's physical measured speeds so it doesn't violently
    // snap.
    if (dt > 0.1) {
      lastSetpoint = currentSpeedsSupplier.get();
      dt = 0.02; // Default to standard 20ms loop time
    }

    // Get raw intent (cubed and deadbanded)
    double rawX = processJoystick(controller.getLeftY()); // Joystick Y is Field X
    double rawY = processJoystick(controller.getLeftX()); // Joystick X is Field Y
    double rawOmega = processJoystick(controller.getRightX());

    // Scale to maximum desired velocities
    double targetVx = rawX * maxLinearVelocity.in(MetersPerSecond);
    double targetVy = rawY * maxLinearVelocity.in(MetersPerSecond);
    double targetOmega = rawOmega * maxAngularVelocity.in(RadiansPerSecond);

    // Apply Anti-Skew 2D Translational Acceleration Limiting
    Translation2d targetTranslation = new Translation2d(targetVx, targetVy);
    Translation2d currentTranslation =
        new Translation2d(lastSetpoint.vxMetersPerSecond, lastSetpoint.vyMetersPerSecond);

    // Find how much the driver is asking to change the velocity
    Translation2d changeVector = targetTranslation.minus(currentTranslation);
    double maxChangeMetersPerSec = maxLinearAcceleration.in(MetersPerSecondPerSecond) * dt;

    Translation2d nextTranslation;
    if (changeVector.getNorm() > maxChangeMetersPerSec) {
      // If the requested change is too aggressive, cap it to the maximum allowed change vector
      // magnitude
      nextTranslation =
          currentTranslation.plus(
              changeVector.times(maxChangeMetersPerSec / changeVector.getNorm()));
    } else {
      nextTranslation = targetTranslation;
    }

    // Apply 1D Rotational Acceleration Limiting
    double maxOmegaChange = maxAngularAcceleration.in(RadiansPerSecondPerSecond) * dt;
    double nextOmega =
        MathUtil.clamp(
            targetOmega,
            lastSetpoint.omegaRadiansPerSecond - maxOmegaChange,
            lastSetpoint.omegaRadiansPerSecond + maxOmegaChange);

    // Save and return new generated setpoint
    lastSetpoint = new ChassisSpeeds(nextTranslation.getX(), nextTranslation.getY(), nextOmega);
    return lastSetpoint;
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
