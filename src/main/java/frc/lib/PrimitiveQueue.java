// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

import java.util.concurrent.atomic.AtomicInteger;

/**
 * A Thread-Safe, GC-Free SPSC (Single-Producer Single-Consumer) Ring Buffer for primitive doubles.
 * This implementation allows the 200Hz odometry thread and 50Hz main thread to communicate without
 * locks.
 */
public class PrimitiveQueue {
  private static final int CAPACITY = 64; // Must be a power of two
  private static final int MASK = CAPACITY - 1;

  /** The array holding the primitive doubles. */
  public final double[] data = new double[CAPACITY];

  // Next index to write to (controlled by 200Hz producer)
  private final AtomicInteger head = new AtomicInteger(0);
  // Next index to read from (controlled by 50Hz consumer)
  private final AtomicInteger tail = new AtomicInteger(0);

  /**
   * Offers a new value to the queue. Safe to call from the high-frequency thread.
   *
   * @param val The value to offer to the queue.
   */
  public void offer(double val) {
    int h = head.get();
    int t = tail.get();

    // If the buffer is full, we drop the oldest data point to maintain real-time consistency
    if (((h + 1) & MASK) == t) {
      return;
    }

    data[h] = val;
    head.lazySet((h + 1) & MASK);
  }

  /**
   * Returns the number of elements currently available to read.
   *
   * @return The number of elements in the queue.
   */
  public int size() {
    return (head.get() - tail.get()) & MASK;
  }

  /**
   * Drains one element from the queue.
   *
   * @return The oldest value in the queue, or 0.0 if empty.
   */
  public double poll() {
    int t = tail.get();
    int h = head.get();

    if (t == h) return 0.0;

    double val = data[t];
    tail.lazySet((t + 1) & MASK);
    return val;
  }

  /** Clears the queue by syncing the tail to the head. */
  public void clear() {
    tail.set(head.get());
  }
}
