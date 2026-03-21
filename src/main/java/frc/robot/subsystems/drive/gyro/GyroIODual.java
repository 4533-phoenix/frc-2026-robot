// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.hardware.GyroType;
import frc.lib.lowlevel.Whacknet;
import frc.robot.subsystems.drive.SparkOdometryThread;

import org.littletonrobotics.junction.Logger;

/**
 * IO implementation that combines inputs from a {@link GyroIONavX} and a {@link GyroIOCanAndGyro}.
 *
 * <p>This implementation prioritizes the NavX (for latency) for raw data but utilizes the
 * CanAndGyro to calculate and correct for gyro drift when the robot is detected to be stationary.
 */
public class GyroIODual implements GyroIO {
  private final GyroIONavX navx = new GyroIONavX(NavXComType.kUSB1);
  private final GyroIOCanAndGyro canandgyro = new GyroIOCanAndGyro();

  private final GyroIOInputs navxIn = new GyroIOInputs();
  private final GyroIOInputs canIn = new GyroIOInputs();

  private volatile Angle driftOffset = Radians.zero();

  private final GyroType[] combinedTypes = new GyroType[] {GyroType.NAVX, GyroType.CANANDGYRO};
  private final int[] combinedActiveFaults = new int[2];
  private final int[] combinedStickyFaults = new int[2];

  public GyroIODual() {
    navx.setWhacknetEnabled(false);
    canandgyro.setWhacknetEnabled(false);

    SparkOdometryThread.getInstance()
        .registerSignal(
            () -> {
              if (Whacknet.getInstance().isLoaded()) {
                double correctedYaw;
                double velocityRadPerSec;

                if (navxIn.connected) {
                  double rawYawRad = navx.getRawYawRad();
                  velocityRadPerSec = navx.getRawVelocityRadPerSec();
                  correctedYaw = rawYawRad + driftOffset.in(Radians);
                } else {
                  correctedYaw = canandgyro.getRawYawRad();
                  velocityRadPerSec = canandgyro.getRawVelocityRadPerSec();
                }

                Whacknet.getInstance()
                    .broadcast(RobotController.getFPGATime(), correctedYaw, velocityRadPerSec);
              }
            });
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Update inputs from both physical sensors
    navx.updateInputs(navxIn);
    canandgyro.updateInputs(canIn);

    if (navxIn.connected) {
      inputs.connected = true;
      Angle currentCorrectedPos = navxIn.yawPosition.plus(driftOffset);

      if (canIn.connected) {
        // Calculate error between NavX (corrected) and CanAndGyro, continuously wrapped
        double errorRad =
            MathUtil.angleModulus(canIn.yawPosition.minus(currentCorrectedPos).in(Radians));
        Angle error = Radians.of(errorRad);
        // Determine if the robot is effectively stationary
        boolean isStill =
            navxIn.yawVelocity.abs(RadiansPerSecond) < VELOCITY_GATE.in(RadiansPerSecond);

        // If stationary and error is significant, adjust the drift offset
        if (isStill && (error.abs(Radians) > ERROR_THRESHOLD.in(Radians))) {
          Angle step = error.times(DRIFT_GAIN);
          // Clamp the correction step to a maximum value to prevent sudden jumps
          step =
              Radians.of(
                  Math.max(
                      -MAX_CORRECTION_PER_FRAME.in(Radians),
                      Math.min(MAX_CORRECTION_PER_FRAME.in(Radians), step.in(Radians))));
          driftOffset = driftOffset.plus(step);
        }
      }

      // Apply the accumulated drift offset to the NavX data
      inputs.yawPosition = navxIn.yawPosition.plus(driftOffset);
      inputs.yawVelocity = navxIn.yawVelocity;
      inputs.odometryYawTimestamps = navxIn.odometryYawTimestamps;

      // Apply drift offset to all high-frequency samples for consistent odometry
      inputs.odometryYawPositions = new double[navxIn.odometryYawPositions.length];
      for (int i = 0; i < navxIn.odometryYawPositions.length; i++) {
        inputs.odometryYawPositions[i] = navxIn.odometryYawPositions[i] + driftOffset.in(Radians);
      }
    } else if (canIn.connected) {
      // Fallback to CanAndGyro if NavX is disconnected
      inputs.connected = true;
      inputs.yawPosition = canIn.yawPosition;
      inputs.yawVelocity = canIn.yawVelocity;
      inputs.odometryYawTimestamps = canIn.odometryYawTimestamps;
      inputs.odometryYawPositions = canIn.odometryYawPositions;
    } else {
      // No gyro data available
      inputs.connected = false;
    }

    // Health
    inputs.healthy = navxIn.healthy && canIn.healthy;
    combinedActiveFaults[0] = navxIn.activeFaults[0];
    combinedStickyFaults[0] = navxIn.stickyFaults[0];
    combinedActiveFaults[1] = canIn.activeFaults[0];
    combinedStickyFaults[1] = canIn.stickyFaults[0];
    inputs.activeFaults = combinedActiveFaults;
    inputs.stickyFaults = combinedStickyFaults;
    inputs.types = combinedTypes;

    // Log the current drift offset
    Logger.recordOutput("Drive/Gyro/DriftOffset", driftOffset);
  }

  @Override
  public void clearFaults() {
    navx.clearFaults();
    canandgyro.clearFaults();
  }
}
