package frc.robot.control.operator;

public interface OperatorProfile {
  /** Returns the rumble intensity for the left joystick. */
  double getLeftRumble();

  /** Returns the rumble intensity for the right joystick. */
  double getRightRumble();

  /** Returns true if the operator wants to deploy the arm. */
  boolean wantsArmDeployment();

  /** Returns true if the operator wants to retract the arm. */
  boolean wantsArmRetraction();

  /** Returns true if the operator wants to intake. */
  boolean wantsIntake();

  /** Returns true if the operator wants to extake. */
  boolean wantsExtake();

  /** Returns true if the operator wants to climb. */
  boolean wantsClimb();

  /** Returns true if the operator wants to raise the climber. */
  boolean wantsClimberUp();

  /** Returns true if the operator wants to lower the climber. */
  boolean wantsClimberDown();
}
