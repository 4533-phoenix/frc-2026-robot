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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.hardware.GyroType;
import frc.lib.lowlevel.Whacknet;
import frc.robot.subsystems.drive.SparkOdometryThread;
import frc.robot.subsystems.drive.SparkOdometryThread.PrimitiveQueue;
import org.littletonrobotics.junction.Logger;

/**
 * IO implementation that combines a Studica NavX and a Redux Canandgyro.
 *
 * <p>This implementation prioritizes the NavX (for low USB latency) for high-frequency odometry and
 * Whacknet broadcasts, but utilizes the Canandgyro to calculate and correct for NavX drift when the
 * robot is detected to be stationary.
 */
public class GyroIODual implements GyroIO {
  private final AHRS navx;
  private final Canandgyro canandgyro;

  private final PrimitiveQueue yawPositionQueue = new PrimitiveQueue();
  private final PrimitiveQueue yawTimestampQueue;

  private volatile Angle driftOffset = Radians.zero();

  private final GyroType[] combinedTypes = new GyroType[] {GyroType.NAVX, GyroType.CANANDGYRO};
  private final int[] combinedActiveFaults = new int[2];
  private final int[] combinedStickyFaults = new int[2];

  public GyroIODual() {
    // Initialize Hardware
    navx = new AHRS(NavXComType.kUSB1, (int) ODOMETRY_FREQUENCY.in(Hertz));
    navx.zeroYaw();

    canandgyro = new Canandgyro(IMU_CAN_ID);
    CanandgyroSettings settings = new CanandgyroSettings();
    settings.setYawFramePeriod(1 / ODOMETRY_FREQUENCY.in(Hertz));
    settings.setAngularVelocityFramePeriod(1 / ODOMETRY_FREQUENCY.in(Hertz));
    canandgyro.setSettings(settings);
    canandgyro.setYaw(0.0);

    // Register single 200Hz signal for Odometry and Whacknet
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    SparkOdometryThread.getInstance()
        .registerSignal(
            () -> {
              boolean navxConnected = navx.isConnected();
              boolean canConnected = canandgyro.isConnected();

              if (!navxConnected && !canConnected) return;

              double yaw, vel;

              // Prioritize NavX, fallback to Canandgyro
              if (navxConnected) {
                double rawYawRad = Units.degreesToRadians(-navx.getAngle());
                vel = Units.degreesToRadians(-navx.getRate());
                yaw = rawYawRad + (vel * NAVX_LATENCY_SEC.in(Seconds)) + driftOffset.in(Radians);
              } else {
                double rawYawRad = Units.rotationsToRadians(canandgyro.getYaw());
                vel = Units.rotationsToRadians(canandgyro.getAngularVelocityYaw());
                yaw = rawYawRad + (vel * CANANDGYRO_LATENCY_SEC.in(Seconds));
              }

              // Push to odometry queue
              yawPositionQueue.offer(yaw);

              // Broadcast to native vision server if loaded
              if (Whacknet.getInstance().isLoaded()) {
                Whacknet.getInstance().broadcast(RobotController.getFPGATime(), yaw, vel);
              }
            });
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    boolean navxConn = navx.isConnected();
    boolean canConn = canandgyro.isConnected();
    inputs.connected = navxConn || canConn;

    // Drift Compensation Math
    if (navxConn && canConn) {
      Angle navxCorrected =
          Radians.of(Units.degreesToRadians(-navx.getAngle()) + driftOffset.in(Radians));
      Angle canPos = Rotations.of(canandgyro.getYaw());
      AngularVelocity navxVel = DegreesPerSecond.of(-navx.getRate());

      // Calculate error between NavX (corrected) and CanAndGyro, continuously wrapped
      double errorRad = MathUtil.angleModulus(canPos.minus(navxCorrected).in(Radians));
      Angle error = Radians.of(errorRad);

      // Determine if the robot is effectively stationary
      boolean isStill = navxVel.abs(RadiansPerSecond) < VELOCITY_GATE.in(RadiansPerSecond);

      // If stationary and error is significant, adjust the drift offset
      if (isStill && (error.abs(Radians) > ERROR_THRESHOLD.in(Radians))) {
        Angle step = error.times(DRIFT_GAIN);
        // Clamp the correction step to a maximum value to prevent sudden jumps
        double clampedStep =
            MathUtil.clamp(
                step.in(Radians),
                -MAX_CORRECTION_PER_FRAME.in(Radians),
                MAX_CORRECTION_PER_FRAME.in(Radians));
        driftOffset = driftOffset.plus(Radians.of(clampedStep));
      }
    }

    // Standard 50Hz Telemetry
    if (navxConn) {
      inputs.yawPosition =
          Radians.of(Units.degreesToRadians(-navx.getAngle()) + driftOffset.in(Radians));
      inputs.yawVelocity = DegreesPerSecond.of(-navx.getRate());
    } else if (canConn) {
      inputs.yawPosition = Rotations.of(canandgyro.getYaw());
      inputs.yawVelocity = RotationsPerSecond.of(canandgyro.getAngularVelocityYaw());
    } else {
      inputs.yawPosition = Radians.zero();
      inputs.yawVelocity = RadiansPerSecond.zero();
    }

    // Health and Faults
    boolean navxCalibrating = navx.isCalibrating();
    boolean canCalibrating = canandgyro.isCalibrating();

    inputs.healthy =
        inputs.connected
            && !navxCalibrating
            && !canCalibrating
            && ((canandgyro.getActiveFaults().faultBitField() & ~0x1) == 0);

    int navxActive = 0;
    if (!navxConn) navxActive |= 0x1;
    if (navxCalibrating) navxActive |= 0x2;
    combinedStickyFaults[0] |= navxActive;
    combinedActiveFaults[0] = navxActive;

    combinedActiveFaults[1] = canandgyro.getActiveFaults().faultBitField() & ~0x1;
    combinedStickyFaults[1] = canandgyro.getStickyFaults().faultBitField() & ~0x1;

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

    // Log the current drift offset
    Logger.recordOutput("Drive/Gyro/DriftOffset", driftOffset);
  }

  @Override
  public void clearFaults() {
    combinedStickyFaults[0] = 0;
    canandgyro.clearStickyFaults();
  }

  @Override
  public void setYaw(Angle yaw) {
    navx.zeroYaw();
    navx.setAngleAdjustment(-yaw.in(Degrees));
    canandgyro.setYaw(yaw.in(Rotations));

    driftOffset = Radians.zero();

    yawPositionQueue.clear();
    yawTimestampQueue.clear();
  }
}
