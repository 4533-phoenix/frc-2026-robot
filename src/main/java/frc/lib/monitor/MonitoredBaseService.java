// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.monitor;

import frc.lib.service.BaseService;

/** The base class for all background data providers that are also monitored for faults. */
public abstract class MonitoredBaseService extends BaseService implements Monitored {

  /** Constructs a MonitoredBaseService with the default name. */
  public MonitoredBaseService() {
    super();
    MonitorRegistry.register(this);
  }

  /**
   * Constructs a MonitoredBaseService with a custom name.
   *
   * @param name The name of the service.
   */
  public MonitoredBaseService(String name) {
    super(name);
    MonitorRegistry.register(this);
  }
}
