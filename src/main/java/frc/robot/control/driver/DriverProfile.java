package frc.robot.control.driver;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DriverProfile {
  /** Returns the rumble intensity for the left joystick. */
  double getLeftRumble();

  /** Returns the rumble intensity for the right joystick. */
  double getRightRumble();

  /** Returns the desired chassis speeds based on joystick input. */
  ChassisSpeeds getDesiredSpeeds();

  /** Returns whether the driver wants to aim. */
  boolean wantsAim();

  /** Returns whether the driver wants to shoot. */
  boolean wantsShoot();

  /** Returns whether the driver wants to reset the robot pose. */
  boolean wantsReset();
}
