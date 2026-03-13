// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

import java.util.ArrayList;
import java.util.List;

/**
 * Utility class for parsing raw fault bitfields from REV hardware into readable strings. Ideal for
 * logging faults via AdvantageKit or SmartDashboard.
 */
public class FaultUtil {
  private FaultUtil() {} // Prevent instantiation

  /**
   * Converts Active/Sticky Faults to Strings
   *
   * @param faults The raw integer representing the fault flags.
   * @return An array of strings describing the active faults, or an empty array if no faults are
   *     active.
   */
  public static String[] getSparkFaults(int faults) {
    if (faults == 0) return new String[0];

    List<String> list = new ArrayList<>();
    if ((faults & 0x1) != 0) list.add("Other");
    if ((faults & 0x2) != 0) list.add("MotorType");
    if ((faults & 0x4) != 0) list.add("Sensor");
    if ((faults & 0x8) != 0) list.add("CAN");
    if ((faults & 0x10) != 0) list.add("Temperature");
    if ((faults & 0x20) != 0) list.add("GateDriver");
    if ((faults & 0x40) != 0) list.add("ESCEEPROM");
    if ((faults & 0x80) != 0) list.add("Firmware");
    return list.toArray(new String[0]);
  }

  /**
   * Converts Active/Sticky Warnings to Strings
   *
   * @param warnings The raw integer representing the warning flags.
   * @return An array of strings describing the active warnings, or an empty array if no warnings
   *     are active.
   */
  public static String[] getSparkWarnings(int warnings) {
    if (warnings == 0) return new String[0];

    List<String> list = new ArrayList<>();
    if ((warnings & 0x1) != 0) list.add("Brownout");
    if ((warnings & 0x2) != 0) list.add("Overcurrent");
    if ((warnings & 0x4) != 0) list.add("ESCEEPROM");
    if ((warnings & 0x8) != 0) list.add("ExtEEPROM");
    if ((warnings & 0x10) != 0) list.add("Sensor");
    if ((warnings & 0x20) != 0) list.add("Stall");
    if ((warnings & 0x40) != 0) list.add("HasReset");
    if ((warnings & 0x80) != 0) list.add("Other");
    return list.toArray(new String[0]);
  }

  /**
   * Converts Active PDH Faults to Strings. Active faults include channel breaker faults, brownout,
   * CAN warning, and hardware fault.
   *
   * @param faults The raw integer representing the active fault flags from the PDH.
   * @return An array of strings describing the active faults, or an empty array if no faults are
   *     active.
   */
  public static String[] getPdhActiveFaults(int faults) {
    if (faults == 0) return new String[0];

    List<String> list = new ArrayList<>();

    // Bits 0-23: Channel Breaker Faults
    for (int i = 0; i < 24; i++) {
      if ((faults & (1 << i)) != 0) {
        list.add("Ch" + i + "Breaker");
      }
    }

    if ((faults & 0x1000000) != 0) list.add("Brownout");
    if ((faults & 0x2000000) != 0) list.add("CanWarning");
    if ((faults & 0x4000000) != 0) list.add("HardwareFault");

    return list.toArray(new String[0]);
  }

  /**
   * Converts Sticky PDH Faults to Strings. Sticky faults include brownout, CAN warning, CAN bus
   * off, hardware fault, firmware fault, and reset event.
   *
   * @param faults The raw integer representing the sticky fault flags from the PDH.
   * @return An array of strings describing the sticky faults, or an empty array if no faults are
   *     active.
   */
  public static String[] getPdhStickyFaults(int faults) {
    if (faults == 0) return new String[0];

    List<String> list = new ArrayList<>();

    // Bits 0-23: Channel Breaker Faults
    for (int i = 0; i < 24; i++) {
      if ((faults & (1 << i)) != 0) {
        list.add("Ch" + i + "Breaker");
      }
    }

    if ((faults & 0x1000000) != 0) list.add("Brownout");
    if ((faults & 0x2000000) != 0) list.add("CanWarning");
    if ((faults & 0x4000000) != 0) list.add("CanBusOff");
    if ((faults & 0x8000000) != 0) list.add("HardwareFault");
    if ((faults & 0x10000000) != 0) list.add("FirmwareFault");
    if ((faults & 0x20000000) != 0) list.add("HasReset");

    return list.toArray(new String[0]);
  }

  /**
   * Formats an array of strings into a single comma-separated string.
   *
   * @param prefix An optional prefix to prepend to the result. Can be null or empty for no prefix.
   * @param arr The array of strings to format.
   * @return A single string with the array elements joined by commas, or "None"
   */
  public static String getArrayString(String prefix, String[] arr) {
    if (arr == null || arr.length == 0) {
      return "None";
    }
    String joined = String.join(", ", arr);
    return (prefix != null && !prefix.isEmpty()) ? prefix + joined : joined;
  }
}
