// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.util;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Utility class for safely interacting with REVLib Spark devices.
 *
 * <p>Provides methods to check for errors before processing data and to retry configuration
 * commands until successful.
 */
public class SparkUtil {
  private SparkUtil() {} // Prevent instantiation

  /**
   * Processes a double value from a Spark only if the last operation was successful.
   *
   * @param spark The Spark device to check for errors.
   * @param supplier A function to supply the value.
   * @param consumer A function to consume the value if the device is error-free.
   * @return True if the operation was successful, false otherwise.
   */
  public static boolean ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
      return true;
    }
    return false;
  }

  /**
   * Processes multiple double values from a Spark only if all operations were successful.
   *
   * @param spark The Spark device to check for errors.
   * @param suppliers An array of functions to supply the values.
   * @param consumer A function to consume the values if the device is error-free.
   * @return True if all operations were successful, false otherwise.
   */
  public static boolean ifOk(
      SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        return false;
      }
    }
    consumer.accept(values);
    return true;
  }

  /**
   * Processes a boolean value from a Spark only if the last operation was successful.
   *
   * @param spark The Spark device to check for errors.
   * @param supplier A function to supply the value.
   * @param consumer A function to consume the value if the device is error-free.
   * @return True if the operation was successful, false otherwise.
   */
  public static boolean ifOk(SparkBase spark, BooleanSupplier supplier, BooleanConsumer consumer) {
    boolean value = supplier.getAsBoolean();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
      return true;
    }
    return false;
  }

  /**
   * Processes multiple boolean values from a Spark only if all operations were successful.
   *
   * @param spark The Spark device to check for errors.
   * @param suppliers An array of functions to supply the values.
   * @param consumer A function to consume the values if the device is error-free.
   * @return True if all operations were successful, false otherwise.
   */
  public static boolean ifOk(
      SparkBase spark, BooleanSupplier[] suppliers, Consumer<boolean[]> consumer) {
    boolean[] values = new boolean[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsBoolean();
      if (spark.getLastError() != REVLibError.kOk) {
        return false;
      }
    }
    consumer.accept(values);
    return true;
  }

  /**
   * Attempts to run a configuration command until no error is produced or max attempts are reached.
   *
   * @param maxAttempts The maximum number of times to retry the command.
   * @param command A function to execute and check for errors.
   * @return True if the command succeeded within the attempt limit, false otherwise.
   */
  public static boolean tryUntilOk(int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        return true;
      }

      // Delay before retrying to prevent CAN bus spam
      if (i < maxAttempts - 1) {
        Timer.delay(0.010);
      }
    }
    return false;
  }

  /**
   * Creates a base SparkMaxConfig with common settings for our robot, including brake mode, current
   * limit, and voltage compensation. This serves as a starting point for configuring Spark Max
   * motors.
   *
   * @param currentLimit The current limit for the Spark Max motor.
   * @param inverted Whether the motor output should be inverted.
   * @return A configured SparkMaxConfig instance.
   */
  public static SparkMaxConfig createBaseConfig(Current currentLimit, boolean inverted) {
    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) currentLimit.in(Amps))
        .voltageCompensation(12.0)
        .inverted(inverted);

    config
        .signals
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(50)
        .outputCurrentPeriodMs(50)
        .faultsAlwaysOn(true)
        .warningsAlwaysOn(true);

    return config;
  }
}
