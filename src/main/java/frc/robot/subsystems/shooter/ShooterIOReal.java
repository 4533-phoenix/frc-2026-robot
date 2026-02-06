package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX flywheelTalon = new TalonFX(flywheelMotorId);
  private final Servo hoodServo = new Servo(hoodServoChannel);

  private final StatusSignal<Angle> flywheelPosition = flywheelTalon.getPosition();
  private final StatusSignal<AngularVelocity> flywheelVelocity = flywheelTalon.getVelocity();
  private final StatusSignal<Voltage> flywheelAppliedVolts = flywheelTalon.getMotorVoltage();
  private final StatusSignal<Current> flywheelCurrent = flywheelTalon.getStatorCurrent();

  public ShooterIOReal() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelTalon.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, flywheelPosition, flywheelVelocity, flywheelAppliedVolts, flywheelCurrent);

    flywheelTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        flywheelPosition, flywheelVelocity, flywheelAppliedVolts, flywheelCurrent);

    inputs.flywheelConnected =
        BaseStatusSignal.isAllGood(
            flywheelPosition, flywheelVelocity, flywheelAppliedVolts, flywheelCurrent);

    inputs.flywheelPositionRad = flywheelPosition.getValue().in(edu.wpi.first.units.Units.Radians);
    inputs.flywheelVelocityRadPerSec =
        flywheelVelocity.getValue().in(edu.wpi.first.units.Units.RadiansPerSecond);
    inputs.flywheelAppliedVolts =
        flywheelAppliedVolts.getValue().in(edu.wpi.first.units.Units.Volts);
    inputs.flywheelCurrentAmps = flywheelCurrent.getValue().in(edu.wpi.first.units.Units.Amps);

    inputs.hoodPosition = hoodServo.getPosition();
  }

  @Override
  public void setFlywheelVolts(double volts) {
    flywheelTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setHoodPosition(double position) {
    hoodServo.set(position);
  }
}
