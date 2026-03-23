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
import static frc.robot.subsystems.drive.DriveConstants.NAVX_LATENCY_SEC;
import static frc.robot.subsystems.drive.DriveConstants.ODOMETRY_FREQUENCY;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.hardware.GyroType;
import frc.lib.lowlevel.Whacknet;
import frc.robot.subsystems.drive.SparkOdometryThread;
import frc.robot.subsystems.drive.SparkOdometryThread.PrimitiveQueue;

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

  private final GyroType[] types = new GyroType[] {GyroType.NAVX};
  private final int[] activeFaults = new int[1];
  private final int[] stickyFaults = new int[1];

  /** Creates a new GyroIONavX. */
  public GyroIONavX() {
    navX = new AHRS(NavXComType.kMXP_SPI, (int) ODOMETRY_FREQUENCY.in(Hertz));
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    SparkOdometryThread.getInstance()
        .registerSignal(
            () -> {
              if (!navX.isConnected()) return;

              double yawPosition = Units.degreesToRadians(-navX.getAngle());
              double yawVelocity = Units.degreesToRadians(-navX.getRate());
              yawPositionQueue.offer(yawPosition + (yawVelocity * NAVX_LATENCY_SEC.in(Seconds)));

              if (Whacknet.getInstance().isLoaded()) {
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

    int currentActive = 0;
    if (!navX.isConnected()) currentActive |= 0x1;
    if (navX.isCalibrating()) currentActive |= 0x2;
    stickyFaults[0] |= currentActive;
    activeFaults[0] = currentActive;
    inputs.activeFaults = activeFaults;
    inputs.stickyFaults = stickyFaults;
    inputs.types = types;

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
  public void clearFaults() {
    stickyFaults[0] = 0;
  }

  @Override
  public void setYaw(Angle yaw) {
    navX.zeroYaw();
    navX.setAngleAdjustment(-yaw.in(Degrees));

    synchronized (yawPositionQueue) {
      yawPositionQueue.clear();
      yawTimestampQueue.clear();
    }
  }
}
