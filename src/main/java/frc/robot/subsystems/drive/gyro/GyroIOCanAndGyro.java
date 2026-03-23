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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.hardware.GyroType;
import frc.lib.lowlevel.Whacknet;
import frc.robot.subsystems.drive.SparkOdometryThread;
import frc.robot.subsystems.drive.SparkOdometryThread.PrimitiveQueue;

/**
 * IO implementation for the Redux Robotics Canandgyro.
 *
 * <p>This implementation configures the gyro to send data frames at the frequency defined in {@link
 * frc.robot.subsystems.drive.DriveConstants#ODOMETRY_FREQUENCY} and registers these signals with
 * the {@link SparkOdometryThread} for accurate, high-frequency odometry.
 */
public class GyroIOCanAndGyro implements GyroIO {
  private final Canandgyro canAndGyro = new Canandgyro(IMU_CAN_ID);
  private final PrimitiveQueue yawPositionQueue = new PrimitiveQueue();
  private final PrimitiveQueue yawTimestampQueue;

  private final GyroType[] types = new GyroType[] {GyroType.CANANDGYRO};
  private final int[] activeFaults = new int[1];
  private final int[] stickyFaults = new int[1];

  /** Creates a new GyroIOCanAndGyro. */
  public GyroIOCanAndGyro() {
    final CanandgyroSettings settings = new CanandgyroSettings();
    settings.setYawFramePeriod(1 / ODOMETRY_FREQUENCY.in(Hertz));
    settings.setAngularVelocityFramePeriod(1 / ODOMETRY_FREQUENCY.in(Hertz));
    canAndGyro.setSettings(settings);
    canAndGyro.setYaw(0.0);

    // Register signals with the asynchronous odometry thread
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    SparkOdometryThread.getInstance()
        .registerSignal(
            () -> {
              if (!canAndGyro.isConnected()) return;

              double yawPosition = Units.rotationsToRadians(canAndGyro.getYaw());
              double yawVelocity = Units.rotationsToRadians(canAndGyro.getAngularVelocityYaw());
              yawPositionQueue.offer(
                  yawPosition + (yawVelocity * CANANDGYRO_LATENCY_SEC.in(Seconds)));

              if (Whacknet.getInstance().isLoaded()) {
                Whacknet.getInstance()
                    .broadcast(RobotController.getFPGATime(), yawPosition, yawVelocity);
              }
            });
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = canAndGyro.isConnected();
    inputs.yawPosition = Rotations.of(canAndGyro.getYaw());
    inputs.yawVelocity = RotationsPerSecond.of(canAndGyro.getAngularVelocityYaw());
    inputs.healthy =
        inputs.connected
            && !canAndGyro.isCalibrating()
            && canAndGyro.getStickyFaults().faultBitField() == 0;

    activeFaults[0] = canAndGyro.getActiveFaults().faultBitField() & ~0x1;
    stickyFaults[0] = canAndGyro.getStickyFaults().faultBitField() & ~0x1;
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
    canAndGyro.clearStickyFaults();
  }

  @Override
  public void setYaw(Angle yaw) {
    canAndGyro.setYaw(yaw.in(Rotations));

    synchronized (yawPositionQueue) {
      yawPositionQueue.clear();
      yawTimestampQueue.clear();
    }
  }
}
