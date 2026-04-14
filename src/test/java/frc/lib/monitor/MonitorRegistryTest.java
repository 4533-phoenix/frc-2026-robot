// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.monitor;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class MonitorRegistryTest {

  private static class MockMonitor implements Monitored {
    private boolean healthy = true;
    private boolean faultsCleared = false;

    @Override
    public boolean isHealthy() {
      return healthy;
    }

    @Override
    public void clearFaults() {
      faultsCleared = true;
    }

    public void setHealthy(boolean healthy) {
      this.healthy = healthy;
    }

    public boolean isFaultsCleared() {
      return faultsCleared;
    }
  }

  @BeforeEach
  void setUp() {
    MonitorRegistry.getMonitors()
        .clear(); // Need to clear it using reflection or accessing list if it was a protected
    // method. The list is returned, so we can clear it.
  }

  @Test
  void testRegisterAndRetrieve() {
    MockMonitor monitor1 = new MockMonitor();
    MonitorRegistry.register(monitor1);

    assertEquals(1, MonitorRegistry.getMonitors().size());
    assertTrue(MonitorRegistry.getMonitors().contains(monitor1));
  }

  @Test
  void testRegisterNoDuplicates() {
    MockMonitor monitor1 = new MockMonitor();
    MonitorRegistry.register(monitor1);
    MonitorRegistry.register(monitor1); // Register again

    assertEquals(1, MonitorRegistry.getMonitors().size());
  }

  @Test
  void testIsHealthyAllTrue() {
    MockMonitor monitor1 = new MockMonitor();
    MockMonitor monitor2 = new MockMonitor();
    MonitorRegistry.register(monitor1);
    MonitorRegistry.register(monitor2);

    assertTrue(MonitorRegistry.isHealthy());
  }

  @Test
  void testIsHealthyOneFalse() {
    MockMonitor monitor1 = new MockMonitor();
    MockMonitor monitor2 = new MockMonitor();
    monitor2.setHealthy(false);
    MonitorRegistry.register(monitor1);
    MonitorRegistry.register(monitor2);

    assertFalse(MonitorRegistry.isHealthy());
  }

  @Test
  void testClearFaults() {
    MockMonitor monitor1 = new MockMonitor();
    MockMonitor monitor2 = new MockMonitor();
    MonitorRegistry.register(monitor1);
    MonitorRegistry.register(monitor2);

    MonitorRegistry.clearFaults();

    assertTrue(monitor1.isFaultsCleared());
    assertTrue(monitor2.isFaultsCleared());
  }
}
