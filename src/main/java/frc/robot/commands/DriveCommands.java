// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Factory class for creating commands related to the drivetrain subsystem.
 *
 * <p>Provides methods for joystick control, PID-assisted rotation, and characterization routines to
 * tune motor controllers.
 */
public class DriveCommands {

  private DriveCommands() {}

  /** Command to maintain a specific heading using a supplier for the target angle. */
  public static Command headingAim(Drive drive, Supplier<Rotation2d> targetSupplier) {
    return Commands.startEnd(
        () -> drive.setHeadingOverrideSupplier(targetSupplier),
        () -> drive.setHeadingOverrideSupplier(null));
  }

  /**
   * Processes joystick inputs to determine linear velocity, applying deadband and cubing inputs for
   * fine control.
   */
  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double norm = Math.hypot(x, y);
    if (norm < JOYSTICK_DEADBAND) return Translation2d.kZero;

    double deadbanded = MathUtil.applyDeadband(norm, JOYSTICK_DEADBAND);
    double cubed = deadbanded * deadbanded * deadbanded;

    return new Translation2d(x, y).times(cubed / norm);
  }

  /**
   * Field relative drive command using two joysticks for linear and angular control.
   *
   * @param drive The drive subsystem.
   * @param xSupplier Supplier for forward/backward input (-1.0 to 1.0).
   * @param ySupplier Supplier for strafe left/right input (-1.0 to 1.0).
   * @param omegaSupplier Supplier for rotation input (-1.0 to 1.0).
   * @return A command to run the drivetrain based on joystick inputs.
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), JOYSTICK_DEADBAND);

          // Cube rotation value for exponential feel with fine low-speed control
          omega = Math.copySign(omega * omega * omega, omega);

          // Convert to field relative speeds
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  drive.getMaxLinearVelocity().times(linearVelocity.getX()),
                  drive.getMaxLinearVelocity().times(linearVelocity.getY()),
                  drive.getMaxAngularVelocity().times(omega));

          // Flip controls if on the Red alliance
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Measures the velocity feedforward constants (kS and kV) for the drive motors.
   *
   * <p>This command should only be used in voltage control mode to collect raw data.
   *
   * @param drive The drive subsystem.
   * @return A command that ramps voltage to collect characterization data.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<AngularVelocity> velocitySamples = new LinkedList<>();
    List<Voltage> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient and hold voltage at 0 before starting
        Commands.run(
                () -> {
                  drive.runCharacterization(Volts.zero());
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  // Ramp voltage linearly over time
                  Voltage voltage = Volts.of(timer.get() * FF_RAMP_RATE);
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate kS and kV using linear regression
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i).in(RadiansPerSecond);
                    sumY += voltageSamples.get(i).in(Volts);
                    sumXY +=
                        velocitySamples.get(i).in(RadiansPerSecond)
                            * voltageSamples.get(i).in(Volts);
                    sumX2 +=
                        velocitySamples.get(i).in(RadiansPerSecond)
                            * velocitySamples.get(i).in(RadiansPerSecond);
                  }
                  // Linear regression formulas
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /**
   * Measures the robot's wheel radius by spinning in a circle and comparing gyro rotation to
   * encoder rotation.
   *
   * @param drive The drive subsystem.
   * @return A command that spins the robot to calculate effective wheel radius.
   */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter =
        new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE.in(RadiansPerSecondPerSecond));
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY.in(RadiansPerSecond));
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta continuously
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      // Calculate average change in wheel position
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }

                      // Calculate radius: (Angle Delta * Dist to Module) / Wheel Dist Delta
                      double wheelRadius =
                          (state.gyroDelta * DRIVE_BASE_RADIUS.in(Meters)) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  /** Holds state information for wheel radius characterization. */
  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
