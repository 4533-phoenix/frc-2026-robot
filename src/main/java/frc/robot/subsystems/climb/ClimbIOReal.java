// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Real IO implementation for the climb subsystem using a Spark Max motor controller.
 *
 * <p>This implementation configures the Spark Max, monitors physical limit switches connected
 * directly to the controller, and implements software limits to prevent driving past the mechanism
 * limits.
 */
public class ClimbIOReal implements ClimbIO {
  private final SparkMax spark = new SparkMax(canId, MotorType.kBrushed);

  // References to the limit switches directly connected to the Spark Max
  private final SparkLimitSwitch upperLimit = spark.getForwardLimitSwitch();
  private final SparkLimitSwitch lowerLimit = spark.getReverseLimitSwitch();

  // Debouncer to prevent rapidly toggling connection status
  private final Debouncer liftConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /** Creates a new ClimbIOReal and configures the Spark Max. */
  public ClimbIOReal() {
    var liftCfg = new SparkMaxConfig();
    liftCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) motorCurrentLimit.in(Amps))
        .inverted(true)
        .voltageCompensation(12.0);
    liftCfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        5,
        () ->
            spark.configure(
                liftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /** Updates hardware inputs, monitors connectivity, and reads limit switch states. */
  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    boolean sparkOk = true;

    // Safely retrieve telemetry from the motor controller
    sparkOk &=
        ifOk(
            spark,
            new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
            (vals) -> inputs.appliedVoltage = Volts.of(vals[0] * vals[1]));
    sparkOk &= ifOk(spark, spark::getOutputCurrent, (v) -> inputs.appliedCurrent = Amps.of(v));

    // Safely retrieve limit switch states
    sparkOk &=
        ifOk(
            spark,
            new BooleanSupplier[] {upperLimit::isPressed, lowerLimit::isPressed},
            (vals) -> {
              inputs.upperLimit = vals[0];
              inputs.lowerLimit = vals[1];
            });

    // Debounce the connection status to ensure stability
    inputs.connected = liftConnectedDebounce.calculate(sparkOk);
  }

  /**
   * Sets the lift motor voltage, enforcing limit switch safety checks.
   *
   * @param voltage The requested voltage to apply.
   */
  @Override
  public void setLiftVoltage(Voltage voltage) {
    // Assuming normally open switches, we stop if the switch is closed.
    boolean atUpper = upperLimit.isPressed();
    boolean atLower = lowerLimit.isPressed();

    // Stop the motor if trying to move past a limit switch
    if ((voltage.gt(Volts.zero()) && atUpper) || (voltage.lt(Volts.zero()) && atLower)) {
      spark.setVoltage(0.0);
    } else {
      spark.setVoltage(voltage);
    }
  }
}
