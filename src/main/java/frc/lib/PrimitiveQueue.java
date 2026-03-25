// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

/** A GC-Free queue for holding primitive doubles. */
public class PrimitiveQueue {
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
