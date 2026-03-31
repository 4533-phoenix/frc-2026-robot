// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.service;

/** The base class for all background data providers. */
public abstract class BaseService {
  /** The name of the service. */
  protected final String name;

  /** Constructs a BaseService with the default name. */
  public BaseService() {
    this.name = this.getClass().getSimpleName();
    ServiceManager.register(this);
  }

  /**
   * Constructs a BaseService with a custom name. This can be useful for distinguishing multiple
   * instances of the same service type in logs.
   *
   * @param name The name of the service.
   */
  public BaseService(String name) {
    this.name = name;
    ServiceManager.register(this);
  }

  /** Logic to be executed in parallel. Marked 'abstract' so every child MUST implement it. */
  public abstract void update();

  /**
   * Returns the name of the service, which is by default the class name. This can be overridden for
   * more descriptive names.
   *
   * @return The name of the service.
   */
  public String getName() {
    return name;
  }
}
