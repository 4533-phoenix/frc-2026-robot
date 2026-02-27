// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.util.function.BooleanConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SparkUtil {
  /** Processes a value from a Spark only if the value is valid. */
  public static boolean ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
      return true;
    }
    return false;
  }

  /** Processes a value from a Spark only if the value is valid. */
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

  /** Processes a value from a Spark only if the value is valid. */
  public static boolean ifOk(SparkBase spark, BooleanSupplier supplier, BooleanConsumer consumer) {
    boolean value = supplier.getAsBoolean();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
      return true;
    }
    return false;
  }

  /** Processes a value from a Spark only if the value is valid. */
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

  /** Attempts to run the command until no error is produced. */
  public static boolean tryUntilOk(int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        return true;
      }
    }
    return false;
  }
}
