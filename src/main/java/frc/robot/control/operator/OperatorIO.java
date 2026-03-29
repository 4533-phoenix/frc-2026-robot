package frc.robot.control.operator;

import org.littletonrobotics.junction.AutoLog;

public interface OperatorIO {
  @AutoLog
  public static class OperatorIOInputs {
    public boolean armDeployment;
    public boolean armRetraction;
    public boolean intake;
    public boolean extake;
    public boolean climb;
    public boolean climberUp;
    public boolean climberDown;
  }

  /** Updates the inputs based on the active profile. */
  public default void updateInputs(OperatorIOInputs inputs, OperatorProfile profile) {
    if (profile == null) return;
    
    inputs.armDeployment = profile.wantsArmDeployment();
    inputs.armRetraction = profile.wantsArmRetraction();
    inputs.intake = profile.wantsIntake();
    inputs.extake = profile.wantsExtake();
    inputs.climb = profile.wantsClimb();
    inputs.climberUp = profile.wantsClimberUp();
    inputs.climberDown = profile.wantsClimberDown();
  }
}
