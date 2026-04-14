// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.util;

import frc.lib.hardware.GyroType;

/**
 * Utility class for parsing raw fault bitfields from REV hardware into readable strings. Ideal for
 * logging faults via AdvantageKit or SmartDashboard.
 */
public class FaultUtil {
  private FaultUtil() {} // Prevent instantiation

  /**
   * Appends Spark Max faults to a StringBuilder based on bitfields.
   *
   * @param sb The StringBuilder to append faults to.
   * @param faults The integer representing the fault flags.
   */
  public static void appendSparkFaults(StringBuilder sb, int faults) {
    if ((faults & 0x1) != 0) sb.append("Other, ");
    if ((faults & 0x2) != 0) sb.append("MotorType, ");
    if ((faults & 0x4) != 0) sb.append("Sensor, ");
    if ((faults & 0x8) != 0) sb.append("CAN, ");
    if ((faults & 0x10) != 0) sb.append("Temperature, ");
    if ((faults & 0x20) != 0) sb.append("GateDriver, ");
    if ((faults & 0x40) != 0) sb.append("ESCEEPROM, ");
    if ((faults & 0x80) != 0) sb.append("Firmware, ");
    if (sb.length() > 2) sb.setLength(sb.length() - 2);
  }

  /**
   * Appends Spark Max warnings to a StringBuilder based on bitfields.
   *
   * @param sb The StringBuilder to append warnings to.
   * @param warnings The integer representing the warning flags.
   */
  public static void appendSparkWarnings(StringBuilder sb, int warnings) {
    if ((warnings & 0x1) != 0) sb.append("Brownout, ");
    if ((warnings & 0x2) != 0) sb.append("Overcurrent, ");
    if ((warnings & 0x4) != 0) sb.append("ESCEEPROM, ");
    if ((warnings & 0x8) != 0) sb.append("ExtEEPROM, ");
    if ((warnings & 0x10) != 0) sb.append("Sensor, ");
    if ((warnings & 0x20) != 0) sb.append("Stall, ");
    if ((warnings & 0x40) != 0) sb.append("HasReset, ");
    if ((warnings & 0x80) != 0) sb.append("Other, ");
    if (sb.length() > 2) sb.setLength(sb.length() - 2);
  }

  /**
   * Appends PDH faults to a StringBuilder based on bitfields.
   *
   * @param sb The StringBuilder to append faults to.
   * @param faults The integer representing the fault flags.
   * @param isSticky Whether to append sticky faults.
   */
  public static void appendPdhFaults(StringBuilder sb, int faults, boolean isSticky) {
    // Channel Breakers (Same for both)
    for (int i = 0; i < 24; i++) {
      if ((faults & (1 << i)) != 0) sb.append("Ch").append(i).append("Breaker, ");
    }

    // System Faults
    if ((faults & 0x1000000) != 0) sb.append("Brownout, ");
    if ((faults & 0x2000000) != 0) sb.append("CanWarning, ");

    if (isSticky) {
      if ((faults & 0x4000000) != 0) sb.append("CanBusOff, ");
      if ((faults & 0x8000000) != 0) sb.append("HardwareFault, ");
      if ((faults & 0x10000000) != 0) sb.append("FirmwareFault, ");
      if ((faults & 0x20000000) != 0) sb.append("HasReset, ");
    } else {
      if ((faults & 0x4000000) != 0) sb.append("HardwareFault, ");
    }

    if (sb.length() > 2) sb.setLength(sb.length() - 2);
  }

  /**
   * Appends Canandgyro faults to a StringBuilder based on bitfields.
   *
   * @param sb The StringBuilder to append faults to.
   * @param faults The 8-bit integer representing the fault flags.
   */
  public static void appendCanandgyroFaults(StringBuilder sb, int faults) {
    if ((faults & 0x1) != 0) sb.append("PowerCycle, ");
    if ((faults & 0x2) != 0) sb.append("CanIdConflict, ");
    if ((faults & 0x4) != 0) sb.append("CanGeneralError, ");
    if ((faults & 0x8) != 0) sb.append("OutOfTemperatureRange, ");
    if ((faults & 0x10) != 0) sb.append("HardwareFault, ");
    if ((faults & 0x20) != 0) sb.append("Calibrating, ");
    if ((faults & 0x40) != 0) sb.append("AngularVelocitySaturation, ");
    if ((faults & 0x80) != 0) sb.append("AccelerationSaturation, ");

    if (sb.length() > 2) sb.setLength(sb.length() - 2);
  }

  /**
   * Appends NavX faults to a StringBuilder based on bitfields.
   *
   * @param sb The StringBuilder to append faults to.
   * @param faults The integer representing the fault flags.
   */
  public static void appendNavXFaults(StringBuilder sb, int faults) {
    if ((faults & 0x1) != 0) sb.append("Disconnected, ");
    if ((faults & 0x2) != 0) sb.append("Calibrating, ");
    if (sb.length() > 2) sb.setLength(sb.length() - 2);
  }

  /**
   * Appends gyro faults to a StringBuilder based on the gyro type and fault flags.
   *
   * @param sb The StringBuilder to append faults to.
   * @param type The type of the gyro.
   * @param faults The integer representing the fault flags.
   */
  public static void appendGyroFaults(StringBuilder sb, GyroType type, int faults) {
    if (faults == 0) return;
    switch (type) {
      case NAVX -> appendNavXFaults(sb, faults);
      case CANANDGYRO -> appendCanandgyroFaults(sb, faults);
    }
  }
}
