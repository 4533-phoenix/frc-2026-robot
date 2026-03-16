// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Hertz;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;

/** A thread for handling Spark odometry updates. */
public class SparkOdometryThread {
  /** A GC-Free queue for holding primitive doubles. */
  public static class PrimitiveQueue {
    /** The array to hold the primitive doubles. */
    public final double[] data = new double[50];

    /** The number of elements in the queue. */
    public int size = 0;

    /**
     * Offers a new value to the queue if there is capacity.
     *
     * @param val The value to offer to the queue.
     */
    public void offer(double val) {
      if (size < 50) data[size++] = val;
    }

    /** Clears the queue by resetting the size to 0. */
    public void clear() {
      size = 0;
    }
  }

  private static SparkOdometryThread instance;

  /**
   * Returns the singleton instance of the SparkOdometryThread, creating it if it does not already
   * exist.
   *
   * @return The SparkOdometryThread instance.
   */
  public static SparkOdometryThread getInstance() {
    if (instance == null) instance = new SparkOdometryThread();
    return instance;
  }

  private final Notifier notifier;
  private boolean isStarted = false;

  private final List<Runnable> signals = new ArrayList<>();
  private final List<PrimitiveQueue> timestampQueues = new ArrayList<>();

  private SparkOdometryThread() {
    notifier = new Notifier(this::updateLoop);
  }

  /**
   * Registers a signal to be captured in the odometry update loop.
   *
   * @param signal A Runnable that captures the desired signal and stores it in a high-frequency
   *     queue.
   */
  public void registerSignal(Runnable signal) {
    if (isStarted) throw new IllegalStateException("Cannot register after start.");
    signals.add(signal);
  }

  /**
   * Provides access to the timestamp queue for modules to synchronize their data with the odometry
   * updates.
   *
   * @return The PrimitiveQueue used for timestamps in the odometry thread.
   */
  public PrimitiveQueue makeTimestampQueue() {
    if (isStarted) throw new IllegalStateException("Cannot register after start.");
    PrimitiveQueue newQueue = new PrimitiveQueue();
    timestampQueues.add(newQueue);
    return newQueue;
  }

  /** Starts the odometry update thread. */
  public void startThread() {
    if (isStarted) return;
    isStarted = true;
    notifier.startPeriodic((1.0) / ODOMETRY_FREQUENCY.in(Hertz));
  }

  private void updateLoop() {
    Drive.odometryLock.lock();
    try {
      double currentTimestampSec = RobotController.getFPGATime() / 1.0e6;

      // Offer the current timestamp to all registered timestamp queues for synchronization
      for (PrimitiveQueue queue : timestampQueues) {
        queue.offer(currentTimestampSec);
      }

      // Snapshot all modules exactly now
      for (Runnable signal : signals) {
        signal.run();
      }
    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
