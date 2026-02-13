// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Angle yawPosition = Radians.of(0.0);
    public AngularVelocity yawVelocity = RadiansPerSecond.of(0.0);
    public double[] odometryYawTimestamps = new double[] {};
    public double[] odometryYawPositions = new double[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
