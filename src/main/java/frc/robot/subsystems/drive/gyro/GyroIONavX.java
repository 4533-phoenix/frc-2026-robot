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
import frc.lib.IMUState;
import frc.lib.hardware.GyroType;

/** IO implementation for the Studica NavX gyro. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX;

  // High-frequency data tracking
  private final HighFreqBuffer yawBuffer = new HighFreqBuffer(1);
  private double latestYawRad = 0.0;

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
  public IMUState updateHighFreq(double timestampSec) {
    if (!navX.isConnected()) return null;

    double yawVelocity = Units.degreesToRadians(-navX.getRate());
    double yawPosition = Units.degreesToRadians(-navX.getAngle()) + yawOffset.in(Radians);
    double latencyCompensatedYaw = yawPosition + (yawVelocity * NAVX_LATENCY_SEC.in(Seconds));

    double roll = Units.degreesToRadians(navX.getRoll());
    double pitch = Units.degreesToRadians(navX.getPitch());
    double rollVel = Units.degreesToRadians(navX.getRawGyroX());
    double pitchVel = Units.degreesToRadians(navX.getRawGyroY());

    yawBuffer.offer(timestampSec, latencyCompensatedYaw);
    latestYawRad = latencyCompensatedYaw;

    return isLocked
        ? new IMUState(
            timestampSec, roll, -pitch, latencyCompensatedYaw, rollVel, pitchVel, yawVelocity)
        : null;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.locked = isLocked = inputs.connected && hasBeenSet;

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
    hasBeenSet = true;
  }
}
