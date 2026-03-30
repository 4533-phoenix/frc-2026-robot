// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.service;

import java.util.ArrayList;
import java.util.List;

public class ServiceManager {
  public interface Service {
    /**
     * Logic to be executed every loop. Since this runs in parallel, ensure this method is
     * thread-safe!
     */
    void update();
  }

  private static final List<BaseService> services = new ArrayList<>();

  /** Registers a service to be updated every loop. */
  public static void register(BaseService service) {
    services.add(service);
  }

  /**
   * Executes all registered services in parallel and waits for all to complete. This acts as a
   * synchronous barrier.
   */
  public static void updateAll() {
    services.parallelStream().forEach(BaseService::update);
  }
}
