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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SparkOdometryThread;
import java.util.ArrayList;
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
    inputs.yawPosition = Degrees.of(-navX.getAngle());
    inputs.yawVelocity = DegreesPerSecond.of(-navX.getRawGyroZ());
    inputs.healthy = inputs.connected && !navX.isMagneticDisturbance() && !navX.isCalibrating();

    if (!inputs.healthy) {
      ArrayList<String> reasons = new ArrayList<>();
      if (navX.isMagneticDisturbance()) {
        reasons.add("Magnetic Disturbance");
      }
      if (navX.isCalibrating()) {
        reasons.add("Calibrating");
      }
      if (!navX.isConnected()) {
        reasons.add("Disconnected");
      }
      inputs.unhealthyReasons = reasons.toArray(new String[0]);
    } else if (inputs.unhealthyReasons.length > 0) {
      inputs.unhealthyReasons = new String[0];
    }

    // Empty the queues into the inputs object for logging and odometry processing
    Drive.odometryLock.lock();
    try {
      int count = Math.min(yawTimestampQueue.size(), yawPositionQueue.size());
      inputs.odometryYawTimestamps = new double[count];
      inputs.odometryYawPositions = new double[count];

      for (int i = 0; i < count; i++) {
        Double timestamp = yawTimestampQueue.poll();
        Double angle = yawPositionQueue.poll();
        if (timestamp != null && angle != null) {
          inputs.odometryYawTimestamps[i] = timestamp;
          inputs.odometryYawPositions[i] = Math.toRadians(-angle);
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
