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
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.HighFreqBuffer;
import frc.lib.hardware.GyroType;
import frc.robot.subsystems.drive.Drive.IMUDataConsumer;

/** IO implementation for the Studica NavX gyro. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX;

  // High-frequency data tracking
  private final HighFreqBuffer yawBuffer = new HighFreqBuffer(1);
  private double latestYawRad = 0.0;

  private volatile Angle pitchOffset = Radians.zero();
  private volatile Angle rollOffset = Radians.zero();
  private volatile Angle yawOffset = Radians.zero();

  private volatile boolean isLocked = false;
  private boolean hasBeenSet = false;

  private final GyroType[] types = new GyroType[] {GyroType.NAVX};
  private final int[] activeFaults = new int[1];
  private final int[] stickyFaults = new int[1];

  /** Creates a new GyroIONavX. */
  public GyroIONavX() {
    navX = new AHRS(NavXComType.kMXP_SPI, (int) ODOMETRY_FREQUENCY.in(Hertz));
  }

  @Override
  public void updateHighFreq(double timestampSec, IMUDataConsumer callback) {
    if (!navX.isConnected()) return;

    double latency = NAVX_LATENCY_SEC.in(Seconds);

    // ROLL
    double rollVelocity = Units.degreesToRadians(navX.getRawGyroX());
    double rollPosition = Units.degreesToRadians(navX.getRoll()) + rollOffset.in(Radians);
    double compRoll = rollPosition + (rollVelocity * latency);

    // PITCH
    double pitchVelocity = Units.degreesToRadians(navX.getRawGyroY());
    double pitchPosition = Units.degreesToRadians(navX.getPitch()) + pitchOffset.in(Radians);
    double compPitch = pitchPosition + (pitchVelocity * latency);

    // YAW
    double yawVelocity = Units.degreesToRadians(-navX.getRawGyroZ());
    double yawPosition = Units.degreesToRadians(-navX.getYaw()) + yawOffset.in(Radians);
    double compYaw = yawPosition + (yawVelocity * latency);

    yawBuffer.offer(timestampSec, compYaw);
    latestYawRad = compYaw;

    if (isLocked) {
      callback.accept(
          timestampSec, compRoll, compPitch, compYaw, rollVelocity, pitchVelocity, yawVelocity);
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.locked = isLocked = inputs.connected && hasBeenSet;

    // Reset offset if disconnected
    if (!inputs.connected && hasBeenSet) {
      hasBeenSet = false;
      yawOffset = Radians.zero();
      pitchOffset = Radians.zero();
      rollOffset = Radians.zero();
    }

    // Drain high-frequency yaw measurements
    double[][] tsRef = {inputs.odometryYawTimestamps};
    double[][] yawRef = {inputs.odometryYawPositions};
    yawBuffer.drain(tsRef, yawRef);
    inputs.odometryYawTimestamps = tsRef[0];
    inputs.odometryYawPositions = yawRef[0];

    // Standard telemetry
    if (inputs.odometryYawTimestamps.length > 0) {
      inputs.yawPosition = Radians.of(latestYawRad);
    } else {
      inputs.yawPosition =
          Radians.of(Units.degreesToRadians(-navX.getAngle()) + yawOffset.in(Radians));
    }

    inputs.yawVelocity = DegreesPerSecond.of(-navX.getRate());
    inputs.rollPosition = Degrees.of(navX.getRoll());
    inputs.pitchPosition = Degrees.of(navX.getPitch());
    inputs.rollVelocity = DegreesPerSecond.of(navX.getRawGyroX());
    inputs.pitchVelocity = DegreesPerSecond.of(navX.getRawGyroY());

    inputs.healthy = inputs.connected && !navX.isCalibrating();

    int currentActive = 0;
    if (!navX.isConnected()) currentActive |= 0x1;
    if (navX.isCalibrating()) currentActive |= 0x2;
    stickyFaults[0] |= currentActive;
    activeFaults[0] = currentActive;
    inputs.activeFaults = activeFaults;
    inputs.stickyFaults = stickyFaults;
    inputs.types = types;
  }

  @Override
  public void clearFaults() {
    stickyFaults[0] = 0;
  }

  @Override
  public void setRotation(Rotation3d rotation) {
    yawOffset = Radians.of(rotation.getZ() - Units.degreesToRadians(-navX.getAngle()));
    pitchOffset = Radians.of(rotation.getY() - Units.degreesToRadians(navX.getPitch()));
    rollOffset = Radians.of(rotation.getX() - Units.degreesToRadians(navX.getRoll()));
    hasBeenSet = true;
  }
}
