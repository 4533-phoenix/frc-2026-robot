// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.monitor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The base class for all subsystems that are also monitored for faults. */
public abstract class MonitoredSubsystemBase extends SubsystemBase implements Monitored {

  /** Constructs a MonitoredSubsystemBase. */
  public MonitoredSubsystemBase() {
    super();
    MonitorRegistry.register(this);
  }
}
