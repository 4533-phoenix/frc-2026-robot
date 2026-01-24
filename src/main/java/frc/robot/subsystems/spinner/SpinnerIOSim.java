package frc.robot.subsystems.spinner;

public class SpinnerIOSim implements SpinnerIO {
  private double appliedVolts = 0.0;
  private boolean connected = true;

  @Override
  public void updateInputs(SpinnerIOInputs inputs) {
    inputs.connected = connected;
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void setVoltage(double appliedVolts) {
    this.appliedVolts = appliedVolts;
  }
}
