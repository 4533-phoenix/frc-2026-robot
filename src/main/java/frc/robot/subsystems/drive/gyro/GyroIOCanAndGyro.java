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

/**
 * IO implementation for the Redux Robotics Canandgyro.
 *
 * <p>This implementation configures the gyro to send data frames at the frequency defined in {@link
 * frc.robot.subsystems.drive.DriveConstants#odometryFrequency} and registers these signals with the
 * {@link SparkOdometryThread} for accurate, high-frequency odometry.
 */
public class GyroIOCanAndGyro implements GyroIO {
  private final Canandgyro canandgyro = new Canandgyro(imuCanId);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  /** Creates a new GyroIOCanAndGyro. */
  public GyroIOCanAndGyro() {
    final CanandgyroSettings settings = new CanandgyroSettings();
    // Configure hardware frames to match the desired odometry frequency
    settings.setYawFramePeriod(1 / odometryFrequency.in(Hertz));
    // Set standard less critical velocity data
    settings.setAngularVelocityFramePeriod(1 / odometryLowFrequency.in(Hertz));
    canandgyro.setSettings(settings);
    canandgyro.setYaw(0.0);

    // Register signals with the asynchronous odometry thread
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(canandgyro::getYaw);
  }

  /**
   * Updates the set of loggable inputs, reading from the high-frequency queues.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = canandgyro.isConnected();
    // Hardware returns rotations
    inputs.yawPosition = Rotations.of(canandgyro.getYaw());
    inputs.yawVelocity = RotationsPerSecond.of(canandgyro.getAngularVelocityYaw());

    // Empty the queues into the inputs object for logging and odometry processing
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

    // Clear queues to ensure data is only processed once
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
