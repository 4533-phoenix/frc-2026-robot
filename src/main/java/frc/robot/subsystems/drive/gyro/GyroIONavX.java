// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import frc.robot.subsystems.drive.SparkOdometryThread;
import java.util.Queue;

/**
 * IO implementation for the Studica NavX gyro.
 *
 * <p>This implementation configures the NavX to update at 200Hz via USB and registers its signals
 * with the {@link SparkOdometryThread} for accurate, high-frequency odometry. Note that the NavX
 * returns angles in degrees, which are converted to radians for standard units usage.
 */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(NavXComType.kUSB1, NavXUpdateRate.k200Hz);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  /** Creates a new GyroIONavX. */
  public GyroIONavX() {
    // Register signals with the asynchronous odometry thread
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navX::getAngle);
    navX.zeroYaw();
  }

  /**
   * Updates the set of loggable inputs, reading from the high-frequency queues.
   *
   * @param inputs The inputs object to update.
   */
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    // Hardware returns degrees (with reversed sign), convert to radians
    inputs.yawPosition = Radians.of(Math.toRadians(-navX.getAngle()));
    inputs.yawVelocity = RadiansPerSecond.of(Math.toRadians(-navX.getRawGyroZ()));

    // Empty the queues into the inputs object for logging and odometry processing
    frc.robot.subsystems.drive.Drive.odometryLock.lock();
    try {
      int count = yawTimestampQueue.size();
      inputs.odometryYawTimestamps = new double[count];
      inputs.odometryYawPositions = new double[count];

      int i = 0;
      for (Double timestamp : yawTimestampQueue) {
        inputs.odometryYawTimestamps[i++] = timestamp;
      }

      i = 0;
      for (Double angle : yawPositionQueue) {
        inputs.odometryYawPositions[i++] = Math.toRadians(-angle);
      }

      // Clear queues to ensure data is only processed once
      yawTimestampQueue.clear();
      yawPositionQueue.clear();
    } finally {
      frc.robot.subsystems.drive.Drive.odometryLock.unlock();
    }
  }
}
