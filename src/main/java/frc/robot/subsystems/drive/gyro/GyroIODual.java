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
  private final AHRS navX;
  private final Canandgyro canAndGyro;

  private final PrimitiveQueue yawPositionQueue = new PrimitiveQueue();
  private final PrimitiveQueue yawTimestampQueue;

  // Software offsets to avoid blocking hardware calls
  private volatile Angle navxOffset = Radians.zero();
  private volatile Angle canOffset = Radians.zero();
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

    // Register single signal for Odometry and Whacknet
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    SparkOdometryThread.getInstance()
        .registerSignal(
            () -> {
              boolean navxConnected = navX.isConnected();
              boolean canConnected = canAndGyro.isConnected();

              if (!navxConnected && !canConnected) return;

              double yaw, vel;

              // Prioritize NavX, fallback to Canandgyro
              if (navxConnected) {
                double rawYawRad = Units.degreesToRadians(-navX.getAngle());
                vel = Units.degreesToRadians(-navX.getRate());
                yaw = rawYawRad + navxOffset.in(Radians) + (vel * NAVX_LATENCY_SEC.in(Seconds));
              } else {
                double rawYawRad = Units.rotationsToRadians(canAndGyro.getYaw());
                vel = Units.rotationsToRadians(canAndGyro.getAngularVelocityYaw());
                yaw = rawYawRad + canOffset.in(Radians) + (vel * CANANDGYRO_LATENCY_SEC.in(Seconds));
              }

              // Push to odometry queue
              yawPositionQueue.offer(yaw);

              // Broadcast to native vision server if loaded and we have an established heading
              if (Whacknet.getInstance().isLoaded() && isLocked) {
                Whacknet.getInstance().broadcast(RobotController.getFPGATime(), yaw, vel);
              }
            });
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    boolean navxConn = navX.isConnected();
    boolean canConn = canAndGyro.isConnected();
    boolean navxCalibrating = navX.isCalibrating();
    boolean canCalibrating = canAndGyro.isCalibrating();
    
    inputs.connected = navxConn || canConn;
    inputs.locked = isLocked = inputs.connected && hasBeenSet;

    double rawNavx = Units.degreesToRadians(-navX.getAngle());
    double rawCan = Units.rotationsToRadians(canAndGyro.getYaw());

    // Handle NavX Reconnection (or First Boot Sync)
    if (navxConn && (!navxLastConnected || isFirstUpdate) && !navxCalibrating) {
      if (canConn && !canCalibrating) {
        navxOffset = Radians.of((rawCan + canOffset.in(Radians)) - rawNavx);
        isFirstUpdate = false;
        Logger.recordOutput("Drive/Gyro/Event", "NavX Reconnected - Synced to Canandgyro");
      }
    }

    // Handle Canandgyro Reconnection (or First Boot Sync)
    if (canConn && (!canLastConnected || isFirstUpdate) && !canCalibrating) {
      if (navxConn && !navxCalibrating) {
        canOffset = Radians.of((rawNavx + navxOffset.in(Radians)) - rawCan);
        isFirstUpdate = false;
        Logger.recordOutput("Drive/Gyro/Event", "Canandgyro Reconnected - Synced to NavX");
      }
    }

    navxLastConnected = navxConn;
    canLastConnected = canConn;

    // Drift Compensation Math
    if (navxConn && canConn && !navxCalibrating && !canCalibrating) {
      double navxEffective = rawNavx + navxOffset.in(Radians);
      double canEffective = rawCan + canOffset.in(Radians);
      AngularVelocity navxVel = DegreesPerSecond.of(-navX.getRate());

      // Calculate error between NavX and CanAndGyro, continuously wrapped
      double errorRad = MathUtil.angleModulus(canEffective - navxEffective);

      // Determine if the robot is effectively stationary
      boolean isStill = navxVel.abs(RadiansPerSecond) < VELOCITY_GATE.in(RadiansPerSecond);

      // If stationary and error is significant, slowly pull the NavX offset to match Canandgyro
      if (isStill && (Math.abs(errorRad) > ERROR_THRESHOLD.in(Radians))) {
        double step = errorRad * DRIFT_GAIN;
        double clampedStep = MathUtil.clamp(step, -MAX_CORRECTION_PER_FRAME.in(Radians), MAX_CORRECTION_PER_FRAME.in(Radians));
        navxOffset = navxOffset.plus(Radians.of(clampedStep));
      }
    }

    // Standard 50Hz Telemetry
    if (navxConn) {
      inputs.yawPosition = Radians.of(rawNavx + navxOffset.in(Radians));
      inputs.yawVelocity = DegreesPerSecond.of(-navX.getRate());
    } else if (canConn) {
      inputs.yawPosition = Radians.of(rawCan + canOffset.in(Radians));
      inputs.yawVelocity = RotationsPerSecond.of(canAndGyro.getAngularVelocityYaw());
    } else {
      inputs.yawPosition = Radians.zero();
      inputs.yawVelocity = RadiansPerSecond.zero();
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

    // Log the current offsets to track drift
    Logger.recordOutput("Drive/Gyro/NavXOffset", navxOffset);
    Logger.recordOutput("Drive/Gyro/CanOffset", canOffset);
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
      navxOffset = Radians.of(target - Units.degreesToRadians(-navX.getAngle()));
    }
    if (canAndGyro.isConnected()) {
      canOffset = Radians.of(target - Units.rotationsToRadians(canAndGyro.getYaw()));
    }

    hasBeenSet = true;

    // Clear queues to prevent applying old offsets to pending samples
    synchronized (yawPositionQueue) {
      yawPositionQueue.clear();
      yawTimestampQueue.clear();
    }
  }
}
