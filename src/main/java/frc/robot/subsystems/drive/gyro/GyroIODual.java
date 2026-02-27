// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;

public class GyroIODual implements GyroIO {
  private final GyroIONavX navx = new GyroIONavX();
  private final GyroIOCanAndGyro canandgyro = new GyroIOCanAndGyro();

  private Angle driftOffset = Radians.of(0.0);

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    GyroIOInputs navxIn = new GyroIOInputs();
    GyroIOInputs canIn = new GyroIOInputs();

    navx.updateInputs(navxIn);
    canandgyro.updateInputs(canIn);

    if (navxIn.connected) {
      inputs.connected = true;
      Angle currentCorrectedPos = navxIn.yawPosition.plus(driftOffset);

      if (canIn.connected) {
        Angle error = canIn.yawPosition.minus(currentCorrectedPos);
        boolean isStill =
            navxIn.yawVelocity.abs(RadiansPerSecond) < velocityGate.in(RadiansPerSecond);

        if (isStill && (error.abs(Radians) > errorThreshold.in(Radians))) {
          Angle step = error.times(driftGain);
          step =
              Radians.of(
                  Math.max(
                      -maxCorrectionPerFrame.in(Radians),
                      Math.min(maxCorrectionPerFrame.in(Radians), step.in(Radians))));
          driftOffset = driftOffset.plus(step);
        }
      }

      inputs.yawPosition = navxIn.yawPosition.plus(driftOffset);
      inputs.yawVelocity = navxIn.yawVelocity;
      inputs.odometryYawTimestamps = navxIn.odometryYawTimestamps;

      inputs.odometryYawPositions = new double[navxIn.odometryYawPositions.length];
      for (int i = 0; i < navxIn.odometryYawPositions.length; i++) {
        inputs.odometryYawPositions[i] = navxIn.odometryYawPositions[i] + driftOffset.in(Radians);
      }
    } else if (canIn.connected) {
      inputs.connected = true;
      inputs.yawPosition = canIn.yawPosition;
      inputs.yawVelocity = canIn.yawVelocity;
      inputs.odometryYawTimestamps = canIn.odometryYawTimestamps;
      inputs.odometryYawPositions = canIn.odometryYawPositions;
    } else {
      inputs.connected = false;
    }

    Logger.recordOutput("Drive/Gyro/DriftOffset", driftOffset);
  }
}
