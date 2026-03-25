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
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.IMUState;
import frc.lib.PrimitiveQueue;
import frc.lib.hardware.GyroType;
import org.littletonrobotics.junction.Logger;

/**
 * IO implementation that combines a Studica NavX and a Redux Canandgyro.
 *
 * <p>This implementation prioritizes the NavX (for low USB latency) for high-frequency odometry and
 * Whacknet broadcasts, but utilizes the Canandgyro to calculate and correct for NavX drift across
 * all 3 axes (Yaw, Pitch, Roll) when the robot is detected to be stationary.
 */
public class GyroIODual implements GyroIO {
  private final AHRS navX;
  private final Canandgyro canAndGyro;

  private final PrimitiveQueue yawPositionQueue = new PrimitiveQueue();
  private final PrimitiveQueue yawTimestampQueue = new PrimitiveQueue();

  // Software offsets to avoid blocking hardware calls
  private volatile Angle navxYawOffset = Radians.zero();
  private volatile Angle canYawOffset = Radians.zero();
  private volatile Angle navxRollOffset = Radians.zero();
  private volatile Angle canRollOffset = Radians.zero();
  private volatile Angle navxPitchOffset = Radians.zero();
  private volatile Angle canPitchOffset = Radians.zero();

  private volatile boolean isLocked = false;

  private boolean navxLastConnected = false;
  private boolean canLastConnected = false;
  private boolean isFirstUpdate = true;
  private boolean hasBeenSet = false;

  private final GyroType[] combinedTypes = new GyroType[] {GyroType.NAVX, GyroType.CANANDGYRO};
  private final int[] combinedActiveFaults = new int[2];
  private final int[] combinedStickyFaults = new int[2];

  /** Creates a new GyroIODual. */
  public GyroIODual() {
    // Initialize Hardware without zeroing
    navX = new AHRS(NavXComType.kUSB1, (int) ODOMETRY_FREQUENCY.in(Hertz));

    canAndGyro = new Canandgyro(IMU_CAN_ID);
    CanandgyroSettings settings = new CanandgyroSettings();
    settings.setYawFramePeriod(1 / ODOMETRY_FREQUENCY.in(Hertz));
    settings.setAngularVelocityFramePeriod(1 / ODOMETRY_FREQUENCY.in(Hertz));
    canAndGyro.setSettings(settings);
  }

  @Override
  public IMUState updateHighFreq(double timestampSec) {
    boolean navxConnected = navX.isConnected();
    boolean canConnected = canAndGyro.isConnected();

    if (!navxConnected && !canConnected) return null;

    double yaw, vel, pitch, roll, pitchVel, rollVel;

    if (navxConnected) {
      double rawYawRad = Units.degreesToRadians(-navX.getAngle());
      vel = Units.degreesToRadians(-navX.getRate());
      yaw = rawYawRad + navxYawOffset.in(Radians) + (vel * NAVX_LATENCY_SEC.in(Seconds));

      pitch = Units.degreesToRadians(navX.getPitch()) + navxPitchOffset.in(Radians);
      roll = Units.degreesToRadians(navX.getRoll()) + navxRollOffset.in(Radians);
      pitchVel = Units.degreesToRadians(navX.getRawGyroY());
      rollVel = Units.degreesToRadians(navX.getRawGyroX());
    } else {
      double rawYawRad = Units.rotationsToRadians(canAndGyro.getYaw());
      vel = Units.rotationsToRadians(canAndGyro.getAngularVelocityYaw());
      yaw = rawYawRad + canYawOffset.in(Radians) + (vel * CANANDGYRO_LATENCY_SEC.in(Seconds));

      pitch = Units.rotationsToRadians(canAndGyro.getPitch()) + canPitchOffset.in(Radians);
      roll = Units.rotationsToRadians(canAndGyro.getRoll()) + canRollOffset.in(Radians);
      pitchVel = Units.rotationsToRadians(canAndGyro.getAngularVelocityPitch());
      rollVel = Units.rotationsToRadians(canAndGyro.getAngularVelocityRoll());
    }

    yawPositionQueue.offer(yaw);
    yawTimestampQueue.offer(timestampSec);

    return isLocked ? new IMUState(timestampSec, roll, pitch, yaw, rollVel, pitchVel, vel) : null;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    boolean navxConn = navX.isConnected();
    boolean canConn = canAndGyro.isConnected();
    boolean navxCalibrating = navX.isCalibrating();
    boolean canCalibrating = canAndGyro.isCalibrating();

    inputs.connected = navxConn || canConn;
    inputs.locked = isLocked = inputs.connected && hasBeenSet;

    double rawNavxYaw = Units.degreesToRadians(-navX.getAngle());
    double rawCanYaw = Units.rotationsToRadians(canAndGyro.getYaw());
    double rawNavxRoll = Units.degreesToRadians(navX.getRoll());
    double rawCanRoll = Units.rotationsToRadians(canAndGyro.getRoll());
    double rawNavxPitch = Units.degreesToRadians(navX.getPitch());
    double rawCanPitch = Units.rotationsToRadians(canAndGyro.getPitch());

    // Handle NavX Reconnection (or First Boot Sync)
    if (navxConn && (!navxLastConnected || isFirstUpdate) && !navxCalibrating) {
      if (canConn && !canCalibrating) {
        navxYawOffset = Radians.of((rawCanYaw + canYawOffset.in(Radians)) - rawNavxYaw);
        navxRollOffset = Radians.of((rawCanRoll + canRollOffset.in(Radians)) - rawNavxRoll);
        navxPitchOffset = Radians.of((rawCanPitch + canPitchOffset.in(Radians)) - rawNavxPitch);
        isFirstUpdate = false;
      }
    }

    // Handle Canandgyro Reconnection (or First Boot Sync)
    if (canConn && (!canLastConnected || isFirstUpdate) && !canCalibrating) {
      if (navxConn && !navxCalibrating) {
        canYawOffset = Radians.of((rawNavxYaw + navxYawOffset.in(Radians)) - rawCanYaw);
        canRollOffset = Radians.of((rawNavxRoll + navxRollOffset.in(Radians)) - rawCanRoll);
        canPitchOffset = Radians.of((rawNavxPitch + navxPitchOffset.in(Radians)) - rawCanPitch);
        isFirstUpdate = false;
      }
    }

    navxLastConnected = navxConn;
    canLastConnected = canConn;

    // Drift Compensation Math
    if (navxConn && canConn && !navxCalibrating && !canCalibrating) {
      double navxYawVel = Units.degreesToRadians(-navX.getRate());
      double navxRollVel = Units.degreesToRadians(navX.getRawGyroX());
      double navxPitchVel = Units.degreesToRadians(navX.getRawGyroY());

      // Determine if the robot is effectively stationary on all axes
      double gate = VELOCITY_GATE.in(RadiansPerSecond);
      boolean isStill =
          Math.abs(navxYawVel) < gate
              && Math.abs(navxRollVel) < gate
              && Math.abs(navxPitchVel) < gate;

      if (isStill) {
        // Yaw
        double yawErrorRad =
            MathUtil.angleModulus(
                (rawCanYaw + canYawOffset.in(Radians)) - (rawNavxYaw + navxYawOffset.in(Radians)));
        if (Math.abs(yawErrorRad) > ERROR_THRESHOLD.in(Radians)) {
          double clampedStep =
              MathUtil.clamp(
                  yawErrorRad * DRIFT_GAIN,
                  -MAX_CORRECTION_PER_FRAME.in(Radians),
                  MAX_CORRECTION_PER_FRAME.in(Radians));
          navxYawOffset = navxYawOffset.plus(Radians.of(clampedStep));
        }

        // Roll
        double rollErrorRad =
            MathUtil.angleModulus(
                (rawCanRoll + canRollOffset.in(Radians))
                    - (rawNavxRoll + navxRollOffset.in(Radians)));
        if (Math.abs(rollErrorRad) > ERROR_THRESHOLD.in(Radians)) {
          double clampedStep =
              MathUtil.clamp(
                  rollErrorRad * DRIFT_GAIN,
                  -MAX_CORRECTION_PER_FRAME.in(Radians),
                  MAX_CORRECTION_PER_FRAME.in(Radians));
          navxRollOffset = navxRollOffset.plus(Radians.of(clampedStep));
        }

        // Pitch
        double pitchErrorRad =
            MathUtil.angleModulus(
                (rawCanPitch + canPitchOffset.in(Radians))
                    - (rawNavxPitch + navxPitchOffset.in(Radians)));
        if (Math.abs(pitchErrorRad) > ERROR_THRESHOLD.in(Radians)) {
          double clampedStep =
              MathUtil.clamp(
                  pitchErrorRad * DRIFT_GAIN,
                  -MAX_CORRECTION_PER_FRAME.in(Radians),
                  MAX_CORRECTION_PER_FRAME.in(Radians));
          navxPitchOffset = navxPitchOffset.plus(Radians.of(clampedStep));
        }
      }
    }

    // Standard 50Hz Telemetry
    if (navxConn) {
      inputs.yawPosition = Radians.of(rawNavxYaw + navxYawOffset.in(Radians));
      inputs.yawVelocity = DegreesPerSecond.of(-navX.getRate());
      inputs.rollPosition = Radians.of(rawNavxRoll + navxRollOffset.in(Radians));
      inputs.pitchPosition = Radians.of(rawNavxPitch + navxPitchOffset.in(Radians));
      inputs.rollVelocity = DegreesPerSecond.of(navX.getRawGyroX());
      inputs.pitchVelocity = DegreesPerSecond.of(navX.getRawGyroY());
    } else if (canConn) {
      inputs.yawPosition = Radians.of(rawCanYaw + canYawOffset.in(Radians));
      inputs.yawVelocity = RotationsPerSecond.of(canAndGyro.getAngularVelocityYaw());
      inputs.rollPosition = Radians.of(rawCanRoll + canRollOffset.in(Radians));
      inputs.pitchPosition = Radians.of(rawCanPitch + canPitchOffset.in(Radians));
      inputs.rollVelocity = RotationsPerSecond.of(canAndGyro.getAngularVelocityRoll());
      inputs.pitchVelocity = RotationsPerSecond.of(canAndGyro.getAngularVelocityPitch());
    } else {
      inputs.yawPosition = Radians.zero();
      inputs.yawVelocity = RadiansPerSecond.zero();
      inputs.rollPosition = Radians.zero();
      inputs.pitchPosition = Radians.zero();
      inputs.rollVelocity = RadiansPerSecond.zero();
      inputs.pitchVelocity = RadiansPerSecond.zero();
    }

    // Health and Faults
    inputs.healthy =
        inputs.connected
            && !navxCalibrating
            && !canCalibrating
            && ((canAndGyro.getActiveFaults().faultBitField() & ~0x1) == 0);

    int navxActive = 0;
    if (!navxConn) navxActive |= 0x1;
    if (navxCalibrating) navxActive |= 0x2;
    combinedStickyFaults[0] |= navxActive;
    combinedActiveFaults[0] = navxActive;

    combinedActiveFaults[1] = canAndGyro.getActiveFaults().faultBitField() & ~0x1;
    combinedStickyFaults[1] = canAndGyro.getStickyFaults().faultBitField() & ~0x1;

    inputs.activeFaults = combinedActiveFaults;
    inputs.stickyFaults = combinedStickyFaults;
    inputs.types = combinedTypes;

    // Drain 200Hz Queues
    int count = yawTimestampQueue.size;
    if (inputs.odometryYawTimestamps == null || inputs.odometryYawTimestamps.length != count) {
      inputs.odometryYawTimestamps = new double[count];
      inputs.odometryYawPositions = new double[count];
    }

    for (int i = 0; i < count; i++) {
      inputs.odometryYawTimestamps[i] = yawTimestampQueue.data[i];
      inputs.odometryYawPositions[i] = yawPositionQueue.data[i];
    }

    yawTimestampQueue.clear();
    yawPositionQueue.clear();

    // Log the current offsets to track drift across all axes
    Logger.recordOutput("Drive/Gyro/CanOffset/Yaw", canYawOffset);
    Logger.recordOutput("Drive/Gyro/CanOffset/Roll", canRollOffset);
    Logger.recordOutput("Drive/Gyro/CanOffset/Pitch", canPitchOffset);
    Logger.recordOutput("Drive/Gyro/NavXOffset/Yaw", navxYawOffset);
    Logger.recordOutput("Drive/Gyro/NavXOffset/Roll", navxRollOffset);
    Logger.recordOutput("Drive/Gyro/NavXOffset/Pitch", navxPitchOffset);
  }

  @Override
  public void clearFaults() {
    combinedStickyFaults[0] = 0;
    canAndGyro.clearStickyFaults();
  }

  @Override
  public void setYaw(Angle yaw) {
    double target = yaw.in(Radians);

    // Apply the offset mathematically to whichever hardware is currently online
    if (navX.isConnected()) {
      navxYawOffset = Radians.of(target - Units.degreesToRadians(-navX.getAngle()));
    }
    if (canAndGyro.isConnected()) {
      canYawOffset = Radians.of(target - Units.rotationsToRadians(canAndGyro.getYaw()));
    }

    hasBeenSet = true;

    // Clear queues to prevent applying old offsets to pending samples
    synchronized (yawPositionQueue) {
      yawPositionQueue.clear();
      yawTimestampQueue.clear();
    }
  }
}
