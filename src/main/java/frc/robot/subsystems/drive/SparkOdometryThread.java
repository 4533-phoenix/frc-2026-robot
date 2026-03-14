// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.sparktap.SparkTap;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Asynchronous thread that blocks on native CAN hardware frames to read high-frequency measurements
 * to a set of queues.
 */
public class SparkOdometryThread extends Thread {
  private final List<DoubleSupplier> signals = new ArrayList<>();
  private final List<Queue<Double>> signalQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private DoubleSupplier[] bakedSignals = new DoubleSupplier[0];

  @SuppressWarnings("unchecked")
  private Queue<Double>[] bakedSignalQueues = new Queue[0];

  @SuppressWarnings("unchecked")
  private Queue<Double>[] bakedTimestampQueues = new Queue[0];

  private double[] valueBuffer = new double[0];

  private boolean isStarted = false;
  private int syncDeviceId = -1; // The motor we wait for to trigger a loop

  private static SparkOdometryThread instance = null;

  public static SparkOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkOdometryThread();
    }
    return instance;
  }

  private SparkOdometryThread() {
    setName("SparkTap-OdometryThread");
    setDaemon(true); // Don't block JVM shutdown
  }

  /** Sets the CAN ID of the motor to block and wait for (e.g., Front Left Drive). */
  public void setSyncDevice(int canId) {
    this.syncDeviceId = canId;
  }

  @SuppressWarnings("unchecked")
  public void startThread() {
    Drive.odometryLock.lock();
    try {
      if (isStarted) return;

      bakedSignals = signals.toArray(new DoubleSupplier[0]);
      bakedSignalQueues = signalQueues.toArray(new Queue[0]);
      bakedTimestampQueues = timestampQueues.toArray(new Queue[0]);
      valueBuffer = new double[bakedSignals.length];

      isStarted = true;
      super.start();
    } finally {
      Drive.odometryLock.unlock();
    }
  }

  /** Registers a signal (like a SparkTap MotorView method) to be snapshot. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    if (isStarted) throw new IllegalStateException("Cannot register after start.");
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      signals.add(signal);
      signalQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    if (isStarted) throw new IllegalStateException("Cannot register after start.");
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (!isInterrupted()) {
      // Block until the master motor sends a Status 2 frame
      if (syncDeviceId != -1) {
        SparkTap.getInstance().sync(syncDeviceId, SparkTap.Frame.S2);
      } else {
        try {
          Thread.sleep(5);
        } catch (InterruptedException e) {
          break;
        }
      }

      // Read timestamp and all memory-mapped values instantly
      double timestamp = RobotController.getFPGATime() / 1e6;
      for (int i = 0; i < bakedSignals.length; i++) {
        valueBuffer[i] = bakedSignals[i].getAsDouble();
      }

      // 3. Atomically push to queues
      Drive.odometryLock.lock();
      try {
        for (int i = 0; i < bakedSignalQueues.length; i++) {
          bakedSignalQueues[i].offer(valueBuffer[i]);
        }
        for (int i = 0; i < bakedTimestampQueues.length; i++) {
          bakedTimestampQueues[i].offer(timestamp);
        }
      } finally {
        Drive.odometryLock.unlock();
      }
    }
  }
}
