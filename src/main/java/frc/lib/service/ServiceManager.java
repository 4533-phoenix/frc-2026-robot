// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.service;

import java.util.ArrayList;
import java.util.List;

/** Manages the registration and updating of background services. */
public class ServiceManager {
  /** Interface for services that are updated every loop. */
  public interface Service {
    /** Logic to be executed every loop. This runs sequentially on the main thread. */
    void update();
  }

  private static final List<BaseService> services = new ArrayList<>();

  /**
   * Registers a service to be updated every loop.
   *
   * @param service The service to register.
   */
  public static void register(BaseService service) {
    services.add(service);
  }

  /** Executes all registered services sequentially. */
  public static void updateAll() {
    services.forEach(BaseService::update);
  }
}
