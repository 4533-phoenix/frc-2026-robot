// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

/**
 * A buffer for storing synchronized high-frequency data points across multiple signals. Specialized
 * for handling at most 2 data channels (e.g., Gyro Yaw or Swerve Module Drive + Turn).
 */
public class HighFreqBuffer {
  private final PrimitiveQueue dataQueue = new PrimitiveQueue();
  private final int numSignals;
  private final int stride;

  private final double[][] timestampPool = new double[65][];
  private final double[][] valuePool0 = new double[65][];
  private final double[][] valuePool1 = new double[65][];
  private final double[][] valuePool2 = new double[65][];

  /**
   * Creates a new HighFreqBuffer with a specific number of value signals (max 3).
   *
   * @param numSignals The number of data streams (1 to 3).
   */
  public HighFreqBuffer(int numSignals) {
    if (numSignals < 1 || numSignals > 3) {
      throw new IllegalArgumentException("This buffer only supports 1 to 3 signals.");
    }
    this.numSignals = numSignals;
    this.stride = numSignals + 1;

    for (int i = 0; i <= 64; i++) {
      timestampPool[i] = new double[i];
      valuePool0[i] = new double[i];
      if (numSignals >= 2) {
        valuePool1[i] = new double[i];
      }
      if (numSignals == 3) {
        valuePool2[i] = new double[i];
      }
    }
  }

  /**
   * Offers a new data point for a single signal (e.g., Gyro Yaw).
   *
   * @param timestamp The timestamp for the data point.
   * @param v0 The value for the signal.
   */
  public void offer(double timestamp, double v0) {
    if (numSignals != 1) {
      throw new IllegalStateException("Buffer configured for 2 signals, but only 1 offered.");
    }
    // Prevent partial frame desync if the queue is full
    if (dataQueue.size() + stride >= 64) return;

    dataQueue.offer(timestamp);
    dataQueue.offer(v0);
  }

  /**
   * Offers a new data point for two signals (e.g., Swerve Module Drive and Turn).
   *
   * @param timestamp The timestamp for the data point.
   * @param v0 The value for the first signal.
   * @param v1 The value for the second signal.
   */
  public void offer(double timestamp, double v0, double v1) {
    if (numSignals != 2) {
      throw new IllegalStateException("Buffer configured for 2 signals, but wrong number offered.");
    }
    if (dataQueue.size() + stride >= 64) return;

    dataQueue.offer(timestamp);
    dataQueue.offer(v0);
    dataQueue.offer(v1);
  }

  /**
   * Offers a new data point for three signals.
   *
   * @param timestamp The timestamp for the data point.
   * @param v0 The value for the first signal.
   * @param v1 The value for the second signal.
   * @param v2 The value for the third signal.
   */
  public void offer(double timestamp, double v0, double v1, double v2) {
    if (numSignals != 3) {
      throw new IllegalStateException("Buffer configured for 3 signals, but wrong number offered.");
    }
    if (dataQueue.size() + stride >= 64) return;

    dataQueue.offer(timestamp);
    dataQueue.offer(v0);
    dataQueue.offer(v1);
    dataQueue.offer(v2);
  }

  /**
   * Drains the buffer contents into the provided AdvantageKit array wrappers.
   *
   * @param outTimestamps A wrapper (double[1][N]) for the timestamp array.
   * @param outValueWrappers Wrappers for the signals.
   */
  public void drain(double[][] outTimestamps, double[][]... outValueWrappers) {
    int frames = dataQueue.size() / stride;
    if (frames == 0) return;

    // Grab exact size pre-allocated arrays from the pool
    double[] tsArray = timestampPool[frames];
    double[] v0Array = valuePool0[frames];
    double[] v1Array = numSignals >= 2 ? valuePool1[frames] : null;
    double[] v2Array = numSignals == 3 ? valuePool2[frames] : null;

    outTimestamps[0] = tsArray;
    outValueWrappers[0][0] = v0Array;
    if (numSignals >= 2) {
      outValueWrappers[1][0] = v1Array;
    }
    if (numSignals == 3) {
      outValueWrappers[2][0] = v2Array;
    }

    // Poll from Ring Buffer into output arrays without looping through signals
    for (int i = 0; i < frames; i++) {
      tsArray[i] = dataQueue.poll();
      v0Array[i] = dataQueue.poll();

      if (numSignals >= 2) {
        v1Array[i] = dataQueue.poll();
      }
      if (numSignals == 3) {
        v2Array[i] = dataQueue.poll();
      }
    }
  }
}
