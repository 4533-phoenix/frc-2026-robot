package frc.robot.control.operator;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Operator extends SubsystemBase {
  private final OperatorIO io;
  private final OperatorIOInputsAutoLogged inputs = new OperatorIOInputsAutoLogged();
  private final LoggedDashboardChooser<OperatorProfile> chooser;
  private final GenericHID controller;

  public Operator(
      OperatorIO io, LoggedDashboardChooser<OperatorProfile> chooser, GenericHID controller) {
    this.io = io;
    this.chooser = chooser;
    this.controller = controller;
  }

  @Override
  public void periodic() {
    OperatorProfile profile = chooser.get();
    if (profile == null) return;
    io.updateInputs(inputs, profile);
    Logger.processInputs("Operator", inputs);
    Logger.recordOutput("Operator/ActiveProfile", chooser.getSendableChooser().getSelected());

    controller.setRumble(GenericHID.RumbleType.kLeftRumble, profile.getLeftRumble());
    controller.setRumble(GenericHID.RumbleType.kRightRumble, profile.getRightRumble());
  }

// package frc.robot.control.operator;

// import org.littletonrobotics.junction.AutoLog;

// public interface OperatorIO {
//   @AutoLog
//   public static class OperatorIOInputs {
//     public boolean armDeployment;
//     public boolean armRetraction;
//     public boolean intake;
//     public boolean extake;
//     public boolean climb;
//     public boolean climberUp;
//     public boolean climberDown;
//   }

//   /** Updates the inputs based on the active profile. */
//   public default void updateInputs(OperatorIOInputs inputs, OperatorProfile profile) {
//     if (profile == null) return;
    
//     inputs.armDeployment = profile.wantsArmDeployment();
//     inputs.armRetraction = profile.wantsArmRetraction();
//     inputs.intake = profile.wantsIntake();
//     inputs.extake = profile.wantsExtake();
//     inputs.climb = profile.wantsClimb();
//     inputs.climberUp = profile.wantsClimberUp();
//     inputs.climberDown = profile.wantsClimberDown();
//   }
// }

  public Trigger wantsArmDeployment() {
    return new Trigger(() -> inputs.armDeployment);
  }

  public Trigger wantsArmRetraction() {
    return new Trigger(() -> inputs.armRetraction);
  }

  public Trigger wantsIntake() {
    return new Trigger(() -> inputs.intake);
  }

  public Trigger wantsExtake() {
    return new Trigger(() -> inputs.extake);
  }

  public Trigger wantsClimb() {
    return new Trigger(() -> inputs.climb);
  }

  public Trigger wantsClimberUp() {
    return new Trigger(() -> inputs.climberUp);
  }

  public Trigger wantsClimberDown() {
    return new Trigger(() -> inputs.climberDown);
  }
}
