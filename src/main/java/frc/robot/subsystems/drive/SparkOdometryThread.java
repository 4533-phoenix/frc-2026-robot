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
  // reusable buffers to avoid allocating on every sample
  private double[] sparkValueBuffer = new double[8];
  private double[] genericValueBuffer = new double[8];

  private static SparkOdometryThread instance = null;
  private Notifier notifier = new Notifier(this::run);

  public static SparkOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkOdometryThread();
    }
    return instance;
  }

  private SparkOdometryThread() {
    notifier.setName("OdometryThread");
  }

  public void start() {
    if (timestampQueues.size() > 0) {
      notifier.startPeriodic(1.0 / DriveConstants.odometryFrequency.in(Hertz));
    }
  }

  /** Registers a Spark signal to be read from the thread. */
  public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
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

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
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

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  private void run() {
    // Snapshot lists quickly under the lock, then do hardware reads outside the lock
    final List<SparkBase> sparksSnapshot;
    final List<DoubleSupplier> sparkSignalsSnapshot;
    final List<DoubleSupplier> genericSignalsSnapshot;
    final List<Queue<Double>> sparkQueuesSnapshot;
    final List<Queue<Double>> genericQueuesSnapshot;
    final List<Queue<Double>> timestampQueuesSnapshot;

    Drive.odometryLock.lock();
    try {
      sparksSnapshot = new ArrayList<>(sparks);
      sparkSignalsSnapshot = new ArrayList<>(sparkSignals);
      genericSignalsSnapshot = new ArrayList<>(genericSignals);
      sparkQueuesSnapshot = new ArrayList<>(sparkQueues);
      genericQueuesSnapshot = new ArrayList<>(genericQueues);
      timestampQueuesSnapshot = new ArrayList<>(timestampQueues);
    } finally {
      Drive.odometryLock.unlock();
    }

    // Fast exit if nothing registered
    if (timestampQueuesSnapshot.isEmpty()
        && sparkQueuesSnapshot.isEmpty()
        && genericQueuesSnapshot.isEmpty()) {
      return;
    }

    // Read timestamp and sensor values without holding the odometry lock
    double timestamp = RobotController.getFPGATime() / 1e6;

    int sCount = sparkSignalsSnapshot.size();
    if (sparkValueBuffer.length < sCount) {
      sparkValueBuffer = new double[Math.max(sCount, sparkValueBuffer.length * 2)];
    }

    boolean isValid = true;
    for (int i = 0; i < sCount; i++) {
      sparkValueBuffer[i] = sparkSignalsSnapshot.get(i).getAsDouble();
      if (sparksSnapshot.get(i).getLastError() != REVLibError.kOk) {
        isValid = false;
        break; // stop early if a Spark reports an error
      }
    }

    if (!isValid) {
      return;
    }

    int gCount = genericSignalsSnapshot.size();
    if (genericValueBuffer.length < gCount) {
      genericValueBuffer = new double[Math.max(gCount, genericValueBuffer.length * 2)];
    }
    for (int i = 0; i < gCount; i++) {
      genericValueBuffer[i] = genericSignalsSnapshot.get(i).getAsDouble();
    }

    // Offer values into queues while holding the lock briefly to keep updates atomic
    Drive.odometryLock.lock();
    try {
      for (int i = 0; i < sCount; i++) {
        sparkQueuesSnapshot.get(i).offer(sparkValueBuffer[i]);
      }
      for (int i = 0; i < gCount; i++) {
        genericQueuesSnapshot.get(i).offer(genericValueBuffer[i]);
      }
      for (int i = 0; i < timestampQueuesSnapshot.size(); i++) {
        timestampQueuesSnapshot.get(i).offer(timestamp);
      }
    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
