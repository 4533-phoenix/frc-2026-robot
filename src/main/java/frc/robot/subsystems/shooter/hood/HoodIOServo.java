// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.ShooterConstants.hoodServoChannel;

import edu.wpi.first.wpilibj.Servo;

public class HoodIOServo implements HoodIO {
  private final Servo servo = new Servo(hoodServoChannel);

  public HoodIOServo() {
    servo.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = servo.getPosition();
  }

  @Override
  public void setPosition(double position) {
    servo.set(position);
  }
}
