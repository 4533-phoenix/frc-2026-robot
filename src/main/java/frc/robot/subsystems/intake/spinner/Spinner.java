package frc.robot.subsystems.intake.spinner;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.spinner.SpinnerConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Spinner extends SubsystemBase {
  private final SpinnerIO io;
  private final SpinnerIOInputsAutoLogged inputs = new SpinnerIOInputsAutoLogged();

  // Goals for the subsystem
  public enum Goal { STOP, INTAKE, EXTAKE }
  private Goal currentGoal = Goal.STOP;

  private final Alert spinnerDisconnectedAlert =
      new Alert("Intake spinner motor disconnected", AlertType.kWarning);

  public Spinner(SpinnerIO io) {
    this.io = io;
  }

  /** Sets the current requested behavior for the rollers. */
  public void setGoal(Goal goal) { 
    this.currentGoal = goal; 
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spinner", inputs);
    spinnerDisconnectedAlert.set(!inputs.connected);

    // Apply the voltage based on the current goal
    switch (currentGoal) {
      case INTAKE -> io.setVoltage(intakeVoltage);
      case EXTAKE -> io.setVoltage(extakeVoltage);
      case STOP -> io.setVoltage(Volts.of(0.0));
    }

    Logger.recordOutput("Spinner/Goal", currentGoal.toString());
  }
}
