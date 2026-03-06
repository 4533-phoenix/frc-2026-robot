// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake.spinner;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.spinner.SpinnerConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

/**
 * Real IO implementation for the intake using REV SparkMax controllers.
 *
 * <p>This implementation configures the arm motor for position closed-loop control using motion
 * profiling and the spinner motor for velocity control. It optimizes CAN bus traffic by setting
 * specific update frequencies for status signals.
 */
public class SpinnerIOSpark implements SpinnerIO {
  private final SparkMax spark = new SparkMax(canId, MotorType.kBrushless);
  private final RelativeEncoder encoder;

  // Debouncers to prevent rapid flickering of connection status
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /** Creates a new SpinnerIOSpark and configures the SparkMax controllers. */
  public SpinnerIOSpark() {
    encoder = spark.getEncoder();

    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) motorCurrentLimit.in(Amps))
        .voltageCompensation(12.0)
        .inverted(true);
    config
        .encoder
        .positionConversionFactor(internalEncoderPositionFactor)
        .velocityConversionFactor(internalEncoderVelocityFactor);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50);
    tryUntilOk(
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        5,
        () -> {
          double initialPos = encoder.getPosition();
          return encoder.setPosition(initialPos);
        });
  }

  /**
   * Updates inputs by refreshing data from the SparkMax controllers.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(SpinnerIOInputs inputs) {
    // Spinner Motor Inputs
    boolean spinnerSparkOk = true;
    spinnerSparkOk &=
        ifOk(spark, encoder::getVelocity, (value) -> inputs.velocity = RadiansPerSecond.of(value));
    spinnerSparkOk &=
        ifOk(
            spark,
            new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
            (values) -> inputs.appliedVoltage = Volts.of(values[0] * values[1]));
    spinnerSparkOk &=
        ifOk(spark, spark::getOutputCurrent, (value) -> inputs.appliedCurrent = Amps.of(value));

    // Debounce the connection status
    inputs.connected = connectedDebounce.calculate(spinnerSparkOk);
  }

  /**
   * Commands the spinner motor to run at a specific voltage.
   *
   * @param voltage The target voltage for the spinner rollers.
   */
  @Override
  public void setVoltage(Voltage voltage) {
    spark.setVoltage(voltage.in(Volts));
  }
}
