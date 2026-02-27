package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Represents the desired physical state of the shooter subsystem.
 *
 * <p>This record encapsulates the target speed for the flywheels and the target angle for the
 * adjustable hood to achieve a specific shot.
 *
 * @param flywheelSpeed The target angular velocity of the shooter flywheels.
 * @param hoodAngle The target angle of the adjustable hood mechanism.
 */
public record ShooterState(AngularVelocity flywheelSpeed, Angle hoodAngle) {}
