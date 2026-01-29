// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;

/** IO implementation for Canandgyro. */
public class GyroIOCanandgyro implements GyroIO {
  private final Canandgyro canandgyro = new Canandgyro(imuCanId);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOCanandgyro() {
    final CanandgyroSettings settings = new CanandgyroSettings();
    settings.setYawFramePeriod(1 / odometryFrequency);
    settings.setAngularVelocityFramePeriod(1 / 50.0);
    canandgyro.setSettings(settings);
    canandgyro.setYaw(0.0);
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(canandgyro::getYaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = canandgyro.isConnected();
    inputs.yawPosition = canandgyro.getRotation2d();
    inputs.yawVelocityRadPerSec = canandgyro.getAngularVelocityYaw() * 2 * Math.PI;

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
