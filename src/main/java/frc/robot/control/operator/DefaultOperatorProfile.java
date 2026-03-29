package frc.robot.control.operator;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DefaultOperatorProfile implements OperatorProfile {
    private final CommandXboxController controller;
  private final BooleanSupplier isClimbMode;

  public DefaultOperatorProfile(
      CommandXboxController controller,
      BooleanSupplier isClimbMode) {
    this.controller = controller;
    this.isClimbMode = isClimbMode;
  }

  @Override
  public double getLeftRumble() {
    return 0;
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
    return controller.leftBumper().or(controller.rightBumper()).getAsBoolean();
  }

  @Override
  public boolean wantsArmRetraction() {
    return controller.povRight().getAsBoolean();
  }

  @Override
  public boolean wantsIntake() {
    return controller.leftBumper().getAsBoolean();
  }

  @Override
  public boolean wantsExtake() {
    return controller.rightBumper().getAsBoolean();
  }

  @Override
  public boolean wantsClimb() {
    return controller.povLeft().getAsBoolean();
  }

  @Override
  public boolean wantsClimberUp() {
    return controller.povUp().getAsBoolean();
  }

  @Override
  public boolean wantsClimberDown() {
    return controller.povDown().getAsBoolean();
  }
}
