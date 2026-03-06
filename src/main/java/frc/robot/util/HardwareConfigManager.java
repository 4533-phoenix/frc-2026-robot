// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

/**
 * Manages hardware configuration tasks for robot IO subsystems.
 *
 * <p>Allows IO classes to register configuration tasks that are executed sequentially in a
 * background thread at robot startup. Ensures all hardware is initialized before IO classes
 * interact with motors or sensors.
 *
 * <p>Call {@link #startConfigThread()} once at the end of RobotContainer's constructor to begin
 * configuration. Use {@link #isReady()} in IO classes to check if hardware is ready for use.
 */
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

public class HardwareConfigManager {
  // Thread-safe boolean so the main loop knows when hardware is ready
  private static final AtomicBoolean isConfigured = new AtomicBoolean(false);

  // Thread-safe list of configuration tasks so registrations are visible to the config thread
  private static final List<Runnable> configTasks = new CopyOnWriteArrayList<>();

  /**
   * Registers a hardware configuration task to be executed during robot startup.
   *
   * <p>Should be called by IO classes in their constructor to queue configuration logic (e.g., CAN
   * IDs, sensor setup).
   *
   * @param task Runnable containing configuration logic for the hardware device.
   */
  public static void registerTask(Runnable task) {
    if (isConfigured.get()) {
      DriverStation.reportWarning(
          "Tried to register a hardware config task after initialization!", true);
      return;
    }
    configTasks.add(task);
  }

  /**
   * Starts the hardware configuration thread and executes all registered tasks sequentially.
   *
   * <p>Call this exactly ONCE at the end of RobotContainer's constructor. This method launches a
   * background thread that runs all configuration tasks and then marks hardware as ready for use.
   */
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

  /**
   * Checks if hardware configuration is complete and IO classes are allowed to interact with
   * motors/sensors.
   *
   * <p>Should be called by IO classes before performing any hardware operations.
   *
   * @return true if hardware configuration is complete, false otherwise.
   */
  public static boolean isReady() {
    return isConfigured.get();
  }
}
