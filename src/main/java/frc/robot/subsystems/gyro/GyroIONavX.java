// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.gyro;

import static edu.wpi.first.units.Units.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.hardware.GyroType;
import frc.lib.lowlevel.Whacknet;

/**
 * IO implementation for the Studica NavX gyro.
 *
 * <p>This implementation configures the NavX to update at 200Hz via USB and registers its signals
 * with the {@link SparkOdometryThread} for accurate, high-frequency odometry. Note that the NavX
 * returns angles in degrees, which are converted to radians for standard units usage.
 */
public class GyroIONavX implements GyroIO {
  private final AHRS navX;
  private final Notifier notifier;

  private final GyroType[] types = new GyroType[] {GyroType.NAVX};
  private final int[] activeFaults = new int[1];
  private final int[] stickyFaults = new int[1];

  /** Creates a new GyroIONavX. */
  public GyroIONavX() {
    navX = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);
    notifier = new Notifier(this::updateLoop);
    notifier.startPeriodic((1.0) / 200.0);

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
  }

  @Override
  public void clearFaults() {
    stickyFaults[0] = 0;
  }

  @Override
  public void setYaw(Angle yaw) {
    navX.zeroYaw();
    navX.setAngleAdjustment(-yaw.in(Degrees));
  }

  private void updateLoop() {
    if (!navX.isConnected()) return;

    double rollPosition = Units.degreesToRadians(navX.getRoll());
    double rollVelocity = Units.degreesToRadians(navX.getRawGyroX());
    double pitchPosition = Units.degreesToRadians(navX.getPitch());
    double pitchVelocity = Units.degreesToRadians(navX.getRawGyroY());
    double yawPosition = Units.degreesToRadians(-navX.getAngle());
    double yawVelocity = Units.degreesToRadians(-navX.getRawGyroZ());

    if (Whacknet.getInstance().isLoaded()) {
      Whacknet.getInstance()
          .broadcast(
              RobotController.getFPGATime(),
              rollPosition,
              pitchPosition,
              yawPosition,
              rollVelocity,
              pitchVelocity,
              yawVelocity);
    }
  }
}
