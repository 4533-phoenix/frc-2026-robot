// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.control.operator;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.control.GenericControlProfile;
import java.util.function.BooleanSupplier;

/** Default implementation of the OperatorProfile for controlling the robot superstructure. */
public class DefaultOperatorProfile extends GenericControlProfile implements OperatorProfile {
  private final BooleanSupplier isClimbMode;

  /**
   * Constructs a DefaultOperatorProfile.
   *
   * @param controller The Xbox controller used for input.
   * @param isClimbMode BooleanSupplier indicating if climb mode is active.
   */
  public DefaultOperatorProfile(XboxController controller, BooleanSupplier isClimbMode) {
    super(controller);
    this.isClimbMode = isClimbMode;
  }

  @Override
  public double getRightRumble() {
    if (isClimbMode.getAsBoolean()) {
      return 0.25;
    }
    return 0;
  }

  @Override
  public boolean wantsArmDeployment() {
    return wantsIntake() || wantsExtake();
  }

  @Override
  public boolean wantsArmRetraction() {
    return controller.getPOV() == 90;
  }

  @Override
  public boolean wantsIntake() {
    return controller.getLeftBumperButton();
  }

  @Override
  public boolean wantsExtake() {
    return controller.getRightBumperButton();
  }

  @Override
  public boolean wantsClimb() {
    return controller.getPOV() == 270;
  }

  @Override
  public boolean wantsClimberUp() {
    return controller.getPOV() == 0;
  }

  @Override
  public boolean wantsClimberDown() {
    return controller.getPOV() == 180;
  }
}
