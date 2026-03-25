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
import frc.lib.PrimitiveQueue;
import frc.lib.hardware.GyroType;

/** IO implementation for the Redux Robotics Canandgyro. */
public class GyroIOCanAndGyro implements GyroIO {
  private final Canandgyro canAndGyro = new Canandgyro(IMU_CAN_ID);
  private final PrimitiveQueue yawPositionQueue = new PrimitiveQueue();
  private final PrimitiveQueue yawTimestampQueue = new PrimitiveQueue();

  private volatile Angle yawOffset = Radians.zero();
  private volatile boolean isLocked = false;
  private boolean hasBeenSet = false;

  private final GyroType[] types = new GyroType[] {GyroType.CANANDGYRO};
  private final int[] activeFaults = new int[1];
  private final int[] stickyFaults = new int[1];

  /** Creates a new GyroIOCanAndGyro. */
  public GyroIOCanAndGyro() {
    final CanandgyroSettings settings = new CanandgyroSettings();
    settings.setYawFramePeriod(1 / ODOMETRY_FREQUENCY.in(Hertz));
    settings.setAngularVelocityFramePeriod(1 / ODOMETRY_FREQUENCY.in(Hertz));
    canAndGyro.setSettings(settings);
  }

  @Override
  public ImuState updateHighFreq(double timestampSec) {
    if (!canAndGyro.isConnected()) return null;

    double yawVelocity = Units.rotationsToRadians(canAndGyro.getAngularVelocityYaw());
    double yawPosition = Units.rotationsToRadians(canAndGyro.getYaw()) + yawOffset.in(Radians);
    double latencyCompensatedYaw = yawPosition + (yawVelocity * CANANDGYRO_LATENCY_SEC.in(Seconds));

    double roll = Units.rotationsToRadians(canAndGyro.getRoll());
    double pitch = Units.rotationsToRadians(canAndGyro.getPitch());
    double rollVel = Units.rotationsToRadians(canAndGyro.getAngularVelocityRoll());
    double pitchVel = Units.rotationsToRadians(canAndGyro.getAngularVelocityPitch());

    yawPositionQueue.offer(latencyCompensatedYaw);
    yawTimestampQueue.offer(timestampSec);

    return isLocked
        ? new ImuState(
            timestampSec, roll, pitch, latencyCompensatedYaw, rollVel, pitchVel, yawVelocity)
        : null;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = canAndGyro.isConnected();
    inputs.locked = isLocked = inputs.connected && hasBeenSet;
    inputs.yawPosition =
        Radians.of(Units.rotationsToRadians(canAndGyro.getYaw()) + yawOffset.in(Radians));
    inputs.yawVelocity = RotationsPerSecond.of(canAndGyro.getAngularVelocityYaw());
    inputs.rollPosition = Rotations.of(canAndGyro.getRoll());
    inputs.pitchPosition = Rotations.of(canAndGyro.getPitch());
    inputs.rollVelocity = RotationsPerSecond.of(canAndGyro.getAngularVelocityRoll());
    inputs.pitchVelocity = RotationsPerSecond.of(canAndGyro.getAngularVelocityPitch());

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
    yawOffset = Radians.of(yaw.in(Radians) - Units.rotationsToRadians(canAndGyro.getYaw()));
    hasBeenSet = true;

    synchronized (yawPositionQueue) {
      yawPositionQueue.clear();
      yawTimestampQueue.clear();
    }
  }
}
