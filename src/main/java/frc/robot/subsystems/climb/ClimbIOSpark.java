// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.SparkUtil.*;
import static frc.robot.subsystems.climb.ClimbConstants.*;

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
public class ClimbIOSpark implements ClimbIO {
  private final SparkMax spark = new SparkMax(CAN_ID, MotorType.kBrushed);

  // References to the limit switches directly connected to the Spark Max
  private final SparkLimitSwitch upperLimit = spark.getForwardLimitSwitch();
  private final SparkLimitSwitch lowerLimit = spark.getReverseLimitSwitch();

  // Debouncer to prevent rapidly toggling connection status
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /** Creates a new ClimbIOSpark and configures the Spark Max. */
  public ClimbIOSpark() {
    var liftCfg = new SparkMaxConfig();
    liftCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) MOTOR_CURRENT_LIMIT.in(Amps))
        .inverted(true)
        .voltageCompensation(12.0);
    liftCfg
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20)
        .limitsPeriodMs(20);
    tryUntilOk(
        5,
        () ->
            spark.configure(
                liftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

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
    inputs.connected = connectedDebounce.calculate(sparkOk);

    // Health
    inputs.status[0] = spark.getFaults().rawBits;
    inputs.healthy = inputs.status[0] == 0;
    inputs.status[1] = spark.getStickyFaults().rawBits;
    inputs.status[2] = spark.getWarnings().rawBits;
    inputs.status[3] = spark.getStickyWarnings().rawBits;
  }

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

  @Override
  public void clearFaults() {
    spark.clearFaults();
  }
}
