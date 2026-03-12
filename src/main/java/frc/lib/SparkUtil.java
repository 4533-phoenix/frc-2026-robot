// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import edu.wpi.first.util.function.BooleanConsumer;
import java.util.ArrayList;
import java.util.List;
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
    }
    return false;
  }

  /**
   * Converts Active/Sticky Faults to Strings
   *
   * @param faults The Faults object containing the fault flags.
   * @return An array of strings describing the active faults, or an empty array if no faults are
   *     active.
   */
  public static String[] getFaultStrings(Faults faults) {
    List<String> list = new ArrayList<>();
    if (faults.motorType) list.add("MotorType");
    if (faults.sensor) list.add("Sensor");
    if (faults.can) list.add("CAN");
    if (faults.temperature) list.add("Temperature");
    if (faults.gateDriver) list.add("GateDriver");
    if (faults.escEeprom) list.add("ESCEEPROM");
    if (faults.firmware) list.add("Firmware");
    return list.toArray(new String[0]);
  }

  /**
   * Converts Active/Sticky Warnings to Strings
   *
   * @param warnings The Warnings object containing the warning flags.
   * @return An array of strings describing the active warnings, or an empty array if no warnings
   *     are active.
   */
  public static String[] getWarningStrings(Warnings warnings) {
    List<String> list = new ArrayList<>();
    if (warnings.brownout) list.add("Brownout");
    if (warnings.overcurrent) list.add("Overcurrent");
    if (warnings.escEeprom) list.add("ESCEEPROM");
    if (warnings.extEeprom) list.add("ExtEEPROM");
    if (warnings.sensor) list.add("Sensor");
    if (warnings.stall) list.add("Stall");
    if (warnings.hasReset) list.add("HasReset");
    if (warnings.other) list.add("Other");
    return list.toArray(new String[0]);
  }

  /**
   * Formats an array of strings into a single comma-separated string, or "None" if the array is
   * empty.
   *
   * @param prefix A prefix to prepend to the result. Can be empty or null for no
   * @param arr The array of strings to format.
   * @return A comma-separated string or "None" if the array is empty.
   */
  public static String getArrayString(String prefix, String[] arr) {
    if (arr == null || arr.length == 0) {
      return "None";
    }
    if (prefix != null && !prefix.isEmpty()) {
      return prefix + String.join(", ", arr);
    }
    return String.join(", ", arr);
  }
}
