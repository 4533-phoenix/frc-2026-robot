// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class FieldUtilTest {

  @BeforeAll
  public static void setup() {
    // Initialize the WPILib Hardware Abstraction Layer (HAL)
    // This prevents wpiHal.dll from crashing when using DriverStation, Timers, or Alerts
    assert HAL.initialize(500, 0);
  }

  @Test
  public void testHubEnabledLogic() {
    // Setup FMS data for a Red alliance robot where Shift 1 is inactive for Red
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.setGameSpecificMessage("RBB"); // Shift 1 inactive for Red ('R')
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData(); // Force the simulated DS to process the new data

    // At 110 seconds (> 105), it should return !amIInactiveInShift1 -> false
    assertFalse(Util.isHubEnabledAtTime(110.0), "Hub should be inactive > 105s");

    // At 90 seconds (between 80 and 105), it should return amIInactiveInShift1 -> true
    assertTrue(Util.isHubEnabledAtTime(90.0), "Hub should be active between 80s and 105s");

    // End game (<= 30) should always be true
    assertTrue(Util.isHubEnabledAtTime(25.0), "Hub should always be active in endgame");
  }
}
