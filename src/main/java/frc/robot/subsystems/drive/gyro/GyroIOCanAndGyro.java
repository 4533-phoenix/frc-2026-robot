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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SparkOdometryThread;
import java.util.ArrayList;
import java.util.Queue;

/**
 * IO implementation for the Redux Robotics Canandgyro.
 *
 * <p>This implementation configures the gyro to send data frames at the frequency defined in {@link
 * frc.robot.subsystems.drive.DriveConstants#ODOMETRY_FREQUENCY} and registers these signals with
 * the {@link SparkOdometryThread} for accurate, high-frequency odometry.
 */
public class GyroIOCanAndGyro implements GyroIO {
  private final Canandgyro canandgyro = new Canandgyro(IMU_CAN_ID);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  /** Creates a new GyroIOCanAndGyro. */
  public GyroIOCanAndGyro() {
    final CanandgyroSettings settings = new CanandgyroSettings();
    // Configure hardware frames to match the desired odometry frequency
    settings.setYawFramePeriod(1 / ODOMETRY_FREQUENCY.in(Hertz));
    // Set standard less critical velocity data
    settings.setAngularVelocityFramePeriod(1 / ODOMETRY_LOW_FREQUENCY.in(Hertz));
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
    inputs.healthy = inputs.connected && !canandgyro.isCalibrating();

    if (!inputs.healthy) {
      ArrayList<String> reasons = new ArrayList<>();
      if (canandgyro.isCalibrating()) {
        reasons.add("Calibrating");
      }
      if (!canandgyro.isConnected()) {
        reasons.add("Disconnected");
      }
      inputs.unhealthyReasons = reasons.toArray(new String[0]);
    } else if (inputs.unhealthyReasons.length > 0) {
      inputs.unhealthyReasons = new String[0];
    }

    // Empty the queues into the inputs object for logging and odometry processing
    frc.robot.subsystems.drive.Drive.odometryLock.lock();
    try {
      int count = Math.min(yawTimestampQueue.size(), yawPositionQueue.size());
      inputs.odometryYawTimestamps = new double[count];
      inputs.odometryYawPositions = new double[count];

      for (int i = 0; i < count; i++) {
        Double timestamp = yawTimestampQueue.poll();
        Double angle = yawPositionQueue.poll();
        if (timestamp != null && angle != null) {
          inputs.odometryYawTimestamps[i] = timestamp;
          inputs.odometryYawPositions[i] = angle * 2 * Math.PI;
        }
      }

      // Clear any remaining elements in case of mismatch
      yawTimestampQueue.clear();
      yawPositionQueue.clear();
    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
