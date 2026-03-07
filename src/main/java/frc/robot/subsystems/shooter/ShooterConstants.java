// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

/**
 * Physical constants and hardware configurations for the shooter subsystem.
 *
 * <p>Includes data on the game piece, actuator geometry, gear ratios, and motor controller
 * settings.
 */
public final class ShooterConstants {
  // ---------- Ball constants ----------
  /** Diameter of the game piece. */
  public static final Distance ballDiameter = Inches.of(5.91);
  /** Mass of the game piece. */
  public static final Mass ballMass = Grams.of(215.0);
  /** Moment of inertia of the game piece around its center axis. */
  public static final MomentOfInertia ballMomentOfInertia = KilogramSquareMeters.of(4.84e-4);
  /** Coefficient of restitution (bounciness) of the game piece. */
  public static final double ballCOR = 0.58;
  /** Surface friction coefficient of the game piece. */
  public static final double ballSurfaceFriction = 1.0;

  // ---------- Hardware IDs ----------
  /** CAN ID for the flywheel motor controller. */
  public static final int flywheelMotorId = 18;
  /** PWM channel for the hood servo motor. */
  public static final int hoodServoChannel = 0;

  // ---------- Hood geometry & motion ----------
  /** Length of the cranking arm in the hood mechanism. */
  public static final Distance crankArmLength = Inches.of(6.403);
  /** Distance between the pivot points of the hood linkage. */
  public static final Distance groundLinkDistance = Inches.of(7.521);
  /** Offset angle between the crank tangent and the actual launch angle. */
  public static final Angle crankTangentToLaunchAngle = Degrees.of(12.875);
  /** Total angular range of the hood mechanism. */
  public static final Angle mechanismTotalAngle = Degrees.of(149.007);

  /** Minimum physical length of the hood servo actuator. */
  public static final Distance servoMinLength = Inches.of(6.925);
  /** Maximum physical length of the hood servo actuator. */
  public static final Distance servoMaxLength = Inches.of(10.5);
  /** Maximum linear speed of the hood servo actuator. */
  public static final LinearVelocity maxServoVelocity = MetersPerSecond.of(0.02);

  /** Tolerance for the hood servo actuator length. */
  public static final Distance hoodLengthTolerance = Meters.of(0.005);

  // ---------- Flywheel constants ----------
  /** Motor model for the flywheel. */
  public static final DCMotor flywheelGearbox = DCMotor.getFalcon500(1);
  /** Gear reduction ratio for the flywheel (motor to wheel). */
  public static final double flywheelReduction = 1.0;
  /** Moment of inertia of the flywheel assembly. */
  public static final MomentOfInertia flywheelMOI = KilogramSquareMeters.of(0.0042);
  /** Radius of the flywheel driving wheel. */
  public static final Distance flywheelWheelRadius = Inches.of(2.05);
  /** Allowed velocity error before the shooter is considered "ready". */
  public static final AngularVelocity flywheelAngularTolerance = RadiansPerSecond.of(15.0);
  /** Maximum current limit for the flywheel motor. */
  public static final Current flywheelMotorCurrentLimit = Amps.of(60.0);

  // ---------- PID constants for flywheel velocity control ----------
  /** Proportional gain for flywheel velocity control. */
  public static final double flywheelKp = 0.5;
  /** Integral gain for flywheel velocity control. */
  public static final double flywheelKi = 0.0;
  /** Derivative gain for flywheel velocity control. */
  public static final double flywheelKd = 0.0;
  /** Static friction feedforward gain for the flywheel. */
  public static final double flywheelKs = 0.2;
  /** Velocity feedforward gain for the flywheel. */
  public static final double flywheelKv = 0.113;
  /** Acceleration feedforward gain for the flywheel. */
  public static final double flywheelKa = 0.04;

  // ---------- Lobbing constants ----------
  /** Preset shooter state for lobbing game pieces into the coral station. */
  public static final ShooterState lobShootingState =
      new ShooterState(RotationsPerSecond.of(60.0), Degrees.of(70.0));

  // ---------- Aiming / lead constants ----------
  /** Estimated time of flight for the game piece from shooter to target. */
  public static final Time estimatedTimeOfFlight = Seconds.of(1.0);
  /**
   * Position of the shooter on the robot relative to the robot center (robot-frame). X is forward,
   * Y is left.
   */
  public static final Translation2d shooterRobotOffset =
      new Translation2d(Inches.of(-10.25), Inches.of(8.5));
}
