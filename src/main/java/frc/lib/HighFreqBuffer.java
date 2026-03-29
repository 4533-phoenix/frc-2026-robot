// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

/**
 * A buffer for storing synchronized high-frequency data points across multiple signals. Handles N
 * signals per timestamp with zero-allocation performance.
 */
public class HighFreqBuffer {
  private final PrimitiveQueue timestamps = new PrimitiveQueue();
  private final PrimitiveQueue[] valueQueues;

  /**
   * Creates a new HighFreqBuffer with a specific number of value signals.
   *
   * @param numSignals The number of data streams to associate with the timestamp stream.
   */
  public HighFreqBuffer(int numSignals) {
    valueQueues = new PrimitiveQueue[numSignals];
    for (int i = 0; i < numSignals; i++) {
      valueQueues[i] = new PrimitiveQueue();
    }
  }

  /**
   * Offers a new data point for a single signal. Overloaded to avoid varargs array allocation.
   *
   * @param timestamp The timestamp for the data point.
   * @param v0 The value for the first signal.
   */
  public void offer(double timestamp, double v0) {
    timestamps.offer(timestamp);
    valueQueues[0].offer(v0);
  }

  /**
   * Offers a new data point for two signals. Overloaded to avoid varargs array allocation.
   *
   * @param timestamp The timestamp for the data point.
   * @param v0 The value for the first signal.
   * @param v1 The value for the second signal.
   */
  public void offer(double timestamp, double v0, double v1) {
    timestamps.offer(timestamp);
    valueQueues[0].offer(v0);
    valueQueues[1].offer(v1);
  }

  /**
   * Offers a new data point for three signals. Overloaded to avoid varargs array allocation.
   *
   * @param timestamp The timestamp for the data point.
   * @param v0 The value for the first signal.
   * @param v1 The value for the second signal.
   * @param v2 The value for the third signal.
   */
  public void offer(double timestamp, double v0, double v1, double v2) {
    timestamps.offer(timestamp);
    valueQueues[0].offer(v0);
    valueQueues[1].offer(v1);
    valueQueues[2].offer(v2);
  }

  /**
   * Generic offer for N signals. Note: Use specific overloads for 2 or 3 signals to remain 100%
   * GC-free.
   *
   * @param timestamp The timestamp for the data point.
   * @param values The values for the signals.
   */
  public void offer(double timestamp, double[] values) {
    timestamps.offer(timestamp);
    for (int i = 0; i < Math.min(values.length, valueQueues.length); i++) {
      valueQueues[i].offer(values[i]);
    }
  }

  /**
   * Drains the buffer contents into the provided AdvantageKit array wrappers.
   *
   * @param outTimestamps A wrapper (double[1][N]) for the timestamp array.
   * @param outValueWrappers An array of wrappers, one for each signal provided in the constructor.
   */
  public void drain(double[][] outTimestamps, double[][]... outValueWrappers) {
    int count = timestamps.size();
    if (count == 0) return;

    // Ensure output buffers are the correct size
    if (outTimestamps[0] == null || outTimestamps[0].length != count) {
      outTimestamps[0] = new double[count];
    }
    for (int i = 0; i < Math.min(valueQueues.length, outValueWrappers.length); i++) {
      if (outValueWrappers[i][0] == null || outValueWrappers[i][0].length != count) {
        outValueWrappers[i][0] = new double[count];
      }
    }

    // Poll from Ring Buffer into output arrays
    for (int i = 0; i < count; i++) {
      outTimestamps[0][i] = timestamps.poll();
      for (int j = 0; j < Math.min(valueQueues.length, outValueWrappers.length); j++) {
        outValueWrappers[j][0][i] = valueQueues[j].poll();
      }
    }
  }
}
