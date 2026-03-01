// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

public class HardwareConfigManager {
  // Thread-safe boolean so the main loop knows when hardware is ready
  private static final AtomicBoolean isConfigured = new AtomicBoolean(false);

  // Thread-safe list of configuration tasks so registrations are visible to the config thread
  private static final List<Runnable> configTasks = new CopyOnWriteArrayList<>();

  /** IO classes call this in their constructor to queue their configuration. */
  public static void registerTask(Runnable task) {
    if (isConfigured.get()) {
      DriverStation.reportWarning(
          "Tried to register a hardware config task after initialization!", true);
      return;
    }
    configTasks.add(task);
  }

  /** Call this exactly ONCE at the end of RobotContainer's constructor. */
  public static void startConfigThread() {
    Thread configThread =
        new Thread(
            () -> {
              System.out.println("Starting Hardware Configuration...");

              for (int i = 0; i < configTasks.size(); i++) {
                try {
                  // Run each task sequentially
                  configTasks.get(i).run();
                } catch (Exception e) {
                  // Catch silent failures and print them to the Driver Station!
                  DriverStation.reportError(
                      "Hardware Config Exception in task " + i + ": " + e.getMessage(),
                      e.getStackTrace());
                }
              }

              // Unblock the robot!
              isConfigured.set(true);
              System.out.println("Hardware Configuration COMPLETE!");
            });

    configThread.setName("HardwareConfigThread");
    configThread.setPriority(Thread.MIN_PRIORITY); // Don't interrupt the main robot loop
    configThread.start();
  }

  /** Used by IO classes to check if they are allowed to read/write to motors. */
  public static boolean isReady() {
    return isConfigured.get();
  }
}
