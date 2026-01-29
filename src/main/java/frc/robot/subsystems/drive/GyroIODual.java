// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIODual implements GyroIO {
    private final GyroIONavX navx = new GyroIONavX();
    private final GyroIOCanAndGyro canandgyro = new GyroIOCanAndGyro();
    
    private Rotation2d driftOffset = Rotation2d.kZero;

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        GyroIOInputs navxIn = new GyroIOInputs();
        GyroIOInputs canIn = new GyroIOInputs();
        navx.updateInputs(navxIn);
        canandgyro.updateInputs(canIn);

        if (navxIn.connected) {
            inputs.connected = true;
            inputs.yawPosition = navxIn.yawPosition.plus(driftOffset);
            inputs.yawVelocityRadPerSec = navxIn.yawVelocityRadPerSec;
            inputs.odometryYawTimestamps = navxIn.odometryYawTimestamps;
            
            inputs.odometryYawPositions = new Rotation2d[navxIn.odometryYawPositions.length];
            for (int i = 0; i < navxIn.odometryYawPositions.length; i++) {
                inputs.odometryYawPositions[i] = navxIn.odometryYawPositions[i].plus(driftOffset);
            }

            if (canIn.connected) {
                Rotation2d difference = canIn.yawPosition.minus(inputs.yawPosition);
                driftOffset = driftOffset.plus(difference.times(0.01)); 
            }
        } 
        else if (canIn.connected) {
            inputs.connected = true;
            inputs.yawPosition = canIn.yawPosition;
            inputs.yawVelocityRadPerSec = canIn.yawVelocityRadPerSec;
            inputs.odometryYawTimestamps = canIn.odometryYawTimestamps;
            inputs.odometryYawPositions = canIn.odometryYawPositions;
        } 
        else {
            inputs.connected = false;
        }
    }
}