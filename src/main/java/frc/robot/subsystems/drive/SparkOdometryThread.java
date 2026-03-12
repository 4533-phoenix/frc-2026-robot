// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This class uses a {@link Notifier} to sample sensors at a high rate independently of the main
 * robot loop. This is crucial for accurate odometry, as it reduces sampling jitter.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid.
 */
public class SparkOdometryThread {
  private final List<SparkBase> sparks = new ArrayList<>();
  private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> sparkQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  // Baked arrays for zero-allocation iteration in the high-frequency loop
  private SparkBase[] bakedSparks = new SparkBase[0];
  private DoubleSupplier[] bakedSparkSignals = new DoubleSupplier[0];
  private DoubleSupplier[] bakedGenericSignals = new DoubleSupplier[0];

  @SuppressWarnings("unchecked")
  private Queue<Double>[] bakedSparkQueues = new Queue[0];

  @SuppressWarnings("unchecked")
  private Queue<Double>[] bakedGenericQueues = new Queue[0];

  @SuppressWarnings("unchecked")
  private Queue<Double>[] bakedTimestampQueues = new Queue[0];

  // Reusable buffers to avoid allocating on every sample
  private double[] sparkValueBuffer = new double[0];
  private double[] genericValueBuffer = new double[0];

  private boolean isStarted = false;

  private static SparkOdometryThread instance = null;
  private Notifier notifier = new Notifier(this::run);

  /**
   * Returns the singleton instance of the SparkOdometryThread.
   *
   * @return The singleton instance.
   */
  public static SparkOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkOdometryThread();
    }
    return instance;
  }

  private SparkOdometryThread() {
    notifier.setName("OdometryThread");
  }

  /**
   * Starts the high-frequency sampling thread.
   *
   * <p>If no signals have been registered, the thread will not start.
   */
  public void start() {
    Drive.odometryLock.lock();
    try {
      if (isStarted) return;

      // Bake lists into arrays to prevent GC allocations in the run loop
      bakedSparks = sparks.toArray(new SparkBase[0]);
      bakedSparkSignals = sparkSignals.toArray(new DoubleSupplier[0]);
      bakedGenericSignals = genericSignals.toArray(new DoubleSupplier[0]);

      @SuppressWarnings("unchecked")
      Queue<Double>[] sq = sparkQueues.toArray(new Queue[0]);
      bakedSparkQueues = sq;

      @SuppressWarnings("unchecked")
      Queue<Double>[] gq = genericQueues.toArray(new Queue[0]);
      bakedGenericQueues = gq;

      @SuppressWarnings("unchecked")
      Queue<Double>[] tq = timestampQueues.toArray(new Queue[0]);
      bakedTimestampQueues = tq;

      // Size buffers exactly to the registered components
      sparkValueBuffer = new double[bakedSparks.length];
      genericValueBuffer = new double[bakedGenericSignals.length];

      isStarted = true;
    } finally {
      Drive.odometryLock.unlock();
    }

    if (bakedTimestampQueues.length > 0) {
      notifier.startPeriodic(1.0 / DriveConstants.ODOMETRY_FREQUENCY.in(Hertz));
    }
  }

  /**
   * Registers a {@link SparkBase} signal to be read from the thread.
   *
   * <p>The thread checks the {@link SparkBase#getLastError()} for this device. If an error is
   * detected, the entire sample set for that iteration is discarded to ensure validity.
   *
   * @param spark The Spark device to check for errors.
   * @param signal A supplier for the data value (e.g., position or velocity).
   * @return A queue containing the sampled data.
   */
  public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
    if (isStarted) {
      throw new IllegalStateException("Cannot register signals after the thread has started.");
    }
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      sparks.add(spark);
      sparkSignals.add(signal);
      sparkQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /**
   * Registers a generic signal to be read from the thread.
   *
   * <p>This is used for sensors not connected directly to a Spark MAX/FLEX.
   *
   * @param signal A supplier for the data value.
   * @return A queue containing the sampled data.
   */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    if (isStarted) {
      throw new IllegalStateException("Cannot register signals after the thread has started.");
    }
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /**
   * Registers a new queue that will receive timestamp values for each sample.
   *
   * <p>Timestamps are taken via {@link RobotController#getFPGATime()}.
   *
   * @return A queue containing the sample timestamps in seconds.
   */
  public Queue<Double> makeTimestampQueue() {
    if (isStarted) {
      throw new IllegalStateException("Cannot register signals after the thread has started.");
    }
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /**
   * The main logic loop executed by the {@link Notifier} thread.
   *
   * <p>This method snapshots registered signals, reads hardware values without holding the odometry
   * lock (to minimize impact on the main thread), and then offers data to the queues.
   */
  private void run() {
    // Fast exit if nothing registered
    if (bakedTimestampQueues.length == 0
        && bakedSparkQueues.length == 0
        && bakedGenericQueues.length == 0) {
      return;
    }

    // Read timestamp and sensor values without holding the odometry lock
    double timestamp = RobotController.getFPGATime() / 1e6;

    boolean isValid = true;
    for (int i = 0; i < bakedSparkSignals.length; i++) {
      sparkValueBuffer[i] = bakedSparkSignals[i].getAsDouble();
      if (bakedSparks[i].getLastError() != REVLibError.kOk) {
        isValid = false;
        break; // stop early if a Spark reports an error
      }
    }

    if (!isValid) {
      return;
    }

    for (int i = 0; i < bakedGenericSignals.length; i++) {
      genericValueBuffer[i] = bakedGenericSignals[i].getAsDouble();
    }

    // Offer values into queues while holding the lock briefly to keep updates atomic
    Drive.odometryLock.lock();
    try {
      for (int i = 0; i < bakedSparkQueues.length; i++) {
        bakedSparkQueues[i].offer(sparkValueBuffer[i]);
      }
      for (int i = 0; i < bakedGenericQueues.length; i++) {
        bakedGenericQueues[i].offer(genericValueBuffer[i]);
      }
      for (int i = 0; i < bakedTimestampQueues.length; i++) {
        bakedTimestampQueues[i].offer(timestamp);
      }
    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
