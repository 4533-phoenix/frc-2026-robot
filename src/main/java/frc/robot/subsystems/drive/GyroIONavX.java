// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.units.measure.Angle;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte) odometryFrequency.in(Hertz));
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navX::getAngle);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.yawPosition = Radians.of(Math.toRadians(-navX.getAngle()));
    inputs.yawVelocity = RadiansPerSecond.of(Math.toRadians(-navX.getRawGyroZ()));

    int count = yawTimestampQueue.size();
    inputs.odometryYawTimestamps = new double[count];
    inputs.odometryYawPositions = new Angle[count];

    int i = 0;
    for (Double timestamp : yawTimestampQueue) {
      inputs.odometryYawTimestamps[i++] = timestamp;
    }

    i = 0;
    for (Double angle : yawPositionQueue) {
      inputs.odometryYawPositions[i++] = Radians.of(Math.toRadians(-angle));
    }

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
