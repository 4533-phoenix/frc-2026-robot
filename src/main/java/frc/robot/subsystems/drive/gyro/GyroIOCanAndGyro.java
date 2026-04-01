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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.HighFreqBuffer;
import frc.lib.IMUState;
import frc.lib.hardware.GyroType;

/** IO implementation for the Redux Robotics Canandgyro. */
public class GyroIOCanAndGyro implements GyroIO {
  private final Canandgyro canAndGyro = new Canandgyro(IMU_CAN_ID);

  // High-frequency data tracking (1 signal: Yaw)
  private final HighFreqBuffer yawBuffer = new HighFreqBuffer(1);
  private double latestYawRad = 0.0;

  private volatile Angle rollOffset = Radians.zero();
  private volatile Angle pitchOffset = Radians.zero();
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
  public IMUState updateHighFreq(double timestampSec) {
    if (!canAndGyro.isConnected()) return null;

    double latency = CANANDGYRO_LATENCY_SEC.in(Seconds);

    // ROLL
    double rollVelocity = Units.rotationsToRadians(canAndGyro.getAngularVelocityRoll());
    double rollPosition = Units.rotationsToRadians(canAndGyro.getRoll()) + rollOffset.in(Radians);
    double compRoll = rollPosition + (rollVelocity * latency);

    // PITCH
    double pitchVelocity = Units.rotationsToRadians(canAndGyro.getAngularVelocityPitch());
    double pitchPosition =
        Units.rotationsToRadians(canAndGyro.getPitch()) + pitchOffset.in(Radians);
    double compPitch = pitchPosition + (pitchVelocity * latency);

    // YAW
    double yawVelocity = Units.rotationsToRadians(canAndGyro.getAngularVelocityYaw());
    double yawPosition = Units.rotationsToRadians(canAndGyro.getYaw()) + yawOffset.in(Radians);
    double compYaw = yawPosition + (yawVelocity * latency);

    yawBuffer.offer(timestampSec, compYaw);
    latestYawRad = compYaw;

    return isLocked
        ? new IMUState(
            timestampSec, compRoll, compPitch, compYaw, rollVelocity, pitchVelocity, yawVelocity)
        : null;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = canAndGyro.isConnected();
    inputs.locked = isLocked = inputs.connected && hasBeenSet;

    // Drain high-frequency yaw measurements
    double[][] tsRef = {inputs.odometryYawTimestamps};
    double[][] yawRef = {inputs.odometryYawPositions};
    yawBuffer.drain(tsRef, yawRef);
    inputs.odometryYawTimestamps = tsRef[0];
    inputs.odometryYawPositions = yawRef[0];

    // Standard 50Hz Telemetry
    if (inputs.odometryYawTimestamps.length > 0) {
      inputs.yawPosition = Radians.of(latestYawRad);
    } else {
      inputs.yawPosition =
          Radians.of(Units.rotationsToRadians(canAndGyro.getYaw()) + yawOffset.in(Radians));
    }

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
  }

  @Override
  public void clearFaults() {
    canAndGyro.clearStickyFaults();
  }

  @Override
  public void setRotation(Rotation3d rotation) {
    rollOffset = Radians.of(rotation.getX() - Units.rotationsToRadians(canAndGyro.getRoll()));
    pitchOffset = Radians.of(rotation.getY() - Units.rotationsToRadians(canAndGyro.getPitch()));
    yawOffset = Radians.of(rotation.getZ() - Units.rotationsToRadians(canAndGyro.getYaw()));
    hasBeenSet = true;
  }
}
