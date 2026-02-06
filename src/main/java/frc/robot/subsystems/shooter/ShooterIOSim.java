package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              flywheelGearbox, flywheelMomentOfInertia, flywheelReduction),
          flywheelGearbox,
          flywheelReduction);

  private double flywheelAppliedVolts = 0.0;
  private double hoodPosition = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    flywheelSim.setInputVoltage(MathUtil.clamp(flywheelAppliedVolts, -12.0, 12.0));
    flywheelSim.update(0.02);

    inputs.flywheelConnected = true;
    inputs.flywheelPositionRad += flywheelSim.getAngularVelocityRadPerSec() * 0.02;
    inputs.flywheelVelocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.flywheelAppliedVolts = flywheelAppliedVolts;
    inputs.flywheelCurrentAmps = Math.abs(flywheelSim.getCurrentDrawAmps());

    inputs.hoodPosition = hoodPosition;
  }

  @Override
  public void setFlywheelVolts(double volts) {
    flywheelAppliedVolts = volts;
  }

  @Override
  public void setHoodPosition(double position) {
    hoodPosition = position;
  }
}
