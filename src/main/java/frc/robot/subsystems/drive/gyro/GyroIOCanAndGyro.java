// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import frc.robot.subsystems.drive.SparkOdometryThread;
import java.util.Queue;

/** IO implementation for Canandgyro. */
public class GyroIOCanAndGyro implements GyroIO {
  private final Canandgyro canandgyro = new Canandgyro(imuCanId);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOCanAndGyro() {
    final CanandgyroSettings settings = new CanandgyroSettings();
    settings.setYawFramePeriod(1 / odometryFrequency.in(Hertz));
    settings.setAngularVelocityFramePeriod(1 / 50.0);
    canandgyro.setSettings(settings);
    canandgyro.setYaw(0.0);
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(canandgyro::getYaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = canandgyro.isConnected();
    inputs.yawPosition = Radians.of(canandgyro.getYaw() * 2 * Math.PI);
    inputs.yawVelocity = RadiansPerSecond.of(canandgyro.getAngularVelocityYaw() * 2 * Math.PI);

    int count = yawTimestampQueue.size();
    inputs.odometryYawTimestamps = new double[count];
    inputs.odometryYawPositions = new double[count];

    int i = 0;
    for (Double timestamp : yawTimestampQueue) {
      inputs.odometryYawTimestamps[i++] = timestamp;
    }

    i = 0;
    for (Double angle : yawPositionQueue) {
      inputs.odometryYawPositions[i++] = angle * 2 * Math.PI;
    }

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
