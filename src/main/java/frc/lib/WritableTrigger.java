// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.concurrent.atomic.AtomicBoolean;

/** A Trigger that can be manually set to true or false. */
public class WritableTrigger extends Trigger {

  // Holds the underlying true/false state
  private final AtomicBoolean m_state;

  /**
   * Private constructor used to pass the AtomicBoolean to the superclass before the object is fully
   * constructed.
   */
  private WritableTrigger(AtomicBoolean state) {
    super(state::get);
    m_state = state;
  }

  /** Creates a new WritableTrigger with an initial state of false. */
  public WritableTrigger() {
    this(new AtomicBoolean(false));
  }

  /**
   * Creates a new WritableTrigger with the specified initial state.
   *
   * @param initialState The initial value to seed the trigger.
   */
  public WritableTrigger(boolean initialState) {
    this(new AtomicBoolean(initialState));
  }

  /**
   * Sets the state of the trigger. If the state changes, it will fire associated commands on the
   * next loop iteration.
   *
   * @param value The new state.
   */
  public void set(boolean value) {
    m_state.set(value);
  }

  /**
   * Gets the current state of the trigger.
   *
   * @return The current state.
   */
  public boolean get() {
    return m_state.get();
  }

  /**
   * Toggles the state of the trigger.
   *
   * @return The new state after toggling.
   */
  public boolean toggle() {
    m_state.set(!m_state.get());
    return m_state.get();
  }
}
