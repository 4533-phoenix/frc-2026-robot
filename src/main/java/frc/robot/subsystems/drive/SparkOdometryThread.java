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

public class SparkOdometryThread {
  /** A GC-Free queue for holding primitive doubles. */
  public static class PrimitiveQueue {
    public final double[] data = new double[50];
    public int size = 0;

    public void offer(double val) {
      if (size < 50) data[size++] = val;
    }

    public void clear() {
      size = 0;
    }
  }

  private static SparkOdometryThread instance;

  public static SparkOdometryThread getInstance() {
    if (instance == null) instance = new SparkOdometryThread();
    return instance;
  }

  private final Notifier notifier;
  private boolean isStarted = false;

  private final List<Runnable> signals = new ArrayList<>();
  private final PrimitiveQueue timestampQueue = new PrimitiveQueue();

  private SparkOdometryThread() {
    notifier = new Notifier(this::updateLoop);
  }

  public void registerSignal(Runnable signal) {
    if (isStarted) throw new IllegalStateException("Cannot register after start.");
    signals.add(signal);
  }

  public PrimitiveQueue makeTimestampQueue() {
    return timestampQueue;
  }

  public void startThread() {
    if (isStarted) return;
    isStarted = true;
    notifier.startPeriodic((1000.0) / ODOMETRY_FREQUENCY.in(Hertz));
  }

  private void updateLoop() {
    Drive.odometryLock.lock();
    try {
      double currentTimestampSec = RobotController.getFPGATime() / 1.0e6;
      timestampQueue.offer(currentTimestampSec);

      // Snapshot all modules exactly now
      for (Runnable signal : signals) {
        signal.run();
      }
    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
