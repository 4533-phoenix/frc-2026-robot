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
import static frc.robot.subsystems.drive.DriveConstants.ODOMETRY_FREQUENCY;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.drive.SparkOdometryThread;
import frc.robot.subsystems.drive.SparkOdometryThread.PrimitiveQueue;
import frc.robot.util.Whacknet;
import java.util.ArrayList;

/**
 * IO implementation for the Studica NavX gyro.
 *
 * <p>This implementation configures the NavX to update at 200Hz via USB and registers its signals
 * with the {@link SparkOdometryThread} for accurate, high-frequency odometry. Note that the NavX
 * returns angles in degrees, which are converted to radians for standard units usage.
 */
public class GyroIONavX implements GyroIO {
  private final AHRS navX;
  private final PrimitiveQueue yawPositionQueue = new PrimitiveQueue();
  private final PrimitiveQueue yawTimestampQueue;
  private boolean whacknetEnabled = true;

  /**
   * Creates a new GyroIONavX.
   *
   * @param comType The communication type to use for the NavX (e.g., USB, SPI, I2C).
   */
  public GyroIONavX(NavXComType comType) {
    navX = new AHRS(comType, (int) ODOMETRY_FREQUENCY.in(Hertz));
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    SparkOdometryThread.getInstance()
        .registerSignal(
            () -> {
              double yawPosition = getRawYawRad();
              double yawVelocity = getRawVelocityRadPerSec();
              yawPositionQueue.offer(yawPosition + (yawVelocity * 0.001));

              if (whacknetEnabled && Whacknet.getInstance().isLoaded()) {
                Whacknet.getInstance()
                    .broadcast(RobotController.getFPGATime(), yawPosition, yawVelocity);
              }
            });
    navX.zeroYaw();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.yawPosition = Degrees.of(-navX.getAngle());
    inputs.yawVelocity = DegreesPerSecond.of(-navX.getRawGyroZ());
    inputs.healthy = inputs.connected && !navX.isCalibrating();

    if (!inputs.healthy) {
      ArrayList<String> reasons = new ArrayList<>();
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
    int count = yawTimestampQueue.size;
    if (inputs.odometryYawTimestamps == null || inputs.odometryYawTimestamps.length != count) {
      inputs.odometryYawTimestamps = new double[count];
      inputs.odometryYawPositions = new double[count];
    }

    for (int i = 0; i < count; i++) {
      inputs.odometryYawTimestamps[i] = yawTimestampQueue.data[i];
      inputs.odometryYawPositions[i] = yawPositionQueue.data[i];
    }

    yawPositionQueue.clear();
    yawTimestampQueue.clear();
  }

  @Override
  public void setWhacknetEnabled(boolean enabled) {
    this.whacknetEnabled = enabled;
  }

  public double getRawYawRad() {
    return Units.degreesToRadians(-navX.getAngle());
  }

  public double getRawVelocityRadPerSec() {
    return Units.degreesToRadians(-navX.getRate());
  }
}
