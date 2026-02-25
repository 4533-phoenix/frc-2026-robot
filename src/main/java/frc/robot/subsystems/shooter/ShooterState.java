package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public record ShooterState(AngularVelocity flywheelSpeed, Angle hoodAngle) {}
