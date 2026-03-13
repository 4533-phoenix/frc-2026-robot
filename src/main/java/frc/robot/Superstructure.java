// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FieldUtil;
import frc.lib.WritableTrigger;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.arm.Arm;
import frc.robot.subsystems.intake.spinner.Spinner;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterKinematics;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Aiming;
import frc.robot.util.Aiming.AimingResult;
import frc.robot.util.Util;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Superstructure subsystem acts as an orchestrator for the robot's secondary systems.
 *
 * <p>It coordinates the interactions between the Arm, Spinner, Shooter, and Climb subsystems,
 * ensuring that mechanical interlocks are respected (e.g., not deploying the intake while climbing)
 * and managing field-position-based aiming logic.
 */
public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Climb climb;
  private final Arm arm;
  private final Spinner spinner;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;

  // Aiming Suppliers
  private final Supplier<AimingResult> hubAiming;
  private final Supplier<AimingResult> lobAiming;

  // System State Variables
  private final WritableTrigger climbMode = new WritableTrigger(false);
  private AimingResult currentAimingResult = Aiming.noTarget;

  /**
   * Constructs a new Superstructure.
   *
   * @param drive The drive subsystem for pose and velocity data.
   * @param climb The climb subsystem.
   * @param arm The intake arm subsystem.
   * @param spinner The intake spinner subsystem.
   * @param shooter The shooter subsystem.
   * @param indexer The indexer subsystem.
   * @param vision The vision subsystem.
   */
  public Superstructure(Drive drive, Climb climb, Arm arm, Spinner spinner, Shooter shooter, Indexer indexer, Vision vision) {
    this.drive = drive;
    this.climb = climb;
    this.arm = arm;
    this.spinner = spinner;
    this.shooter = shooter;
    this.indexer = indexer;
    this.vision = vision;

    this.hubAiming =
        Aiming.hubAimingSupplier(
            drive::getPose,
            drive::getFieldRelativeVelocity,
            ShooterConstants.SHOOTER_ROBOT_OFFSET,
            ShooterConstants.ESTIMATED_TOF);

    this.lobAiming =
        Aiming.lobAimingSupplier(drive::getPose, ShooterConstants.SHOOTER_ROBOT_OFFSET);
  }

  /**
   * Updates the high-level state of the robot, including aiming calculations based on field
   * position and determining whether the shooter should be active.
   */
  @Override
  public void periodic() {
    boolean isHubEnabled = Util.isHubEnabled();
    Translation2d robotTranslation = drive.getPose().getTranslation();

    // Evaluate aiming and automated shooting states
    if (!climbMode.get()) {
      // Determine if we are in the Hub Shooting zone
      if (FieldUtil.flipAllianceIfNeeded(Constants.SHOOTING_ZONE).contains(robotTranslation)
          && (Util.isHubApproaching() || isHubEnabled)) {
        currentAimingResult = hubAiming.get();
        shooter.setShooterState(
            ShooterKinematics.calculateShooterState(
                Meters.of(currentAimingResult.distanceToTargetMeters())));
      }
      // Determine if we are in the Lobbing zone
      else if (FieldUtil.flipAllianceIfNeeded(Constants.LOBBING_ZONE).contains(robotTranslation)) {
        currentAimingResult = lobAiming.get();
        shooter.setShooterState(ShooterConstants.LOB_STATE);
      }
      // Default state
      else {
        currentAimingResult = Aiming.noTarget;
      }
    } else {
      currentAimingResult = Aiming.noTarget;
    }

    // Determine when the shooter flywheel should actively run during a match
    if (Util.isMatchMode()) {
      if (currentAimingResult.hasTarget() && !climbMode.get()) {
        shooter.setRunning();
      } else {
        shooter.setStop();
      }
    }

    // Logging state to AdvantageKit
    Logger.recordOutput("Superstructure/ClimbMode", climbMode.get());
    Logger.recordOutput("Superstructure/CurrentAimingResult", currentAimingResult);
    Logger.recordOutput("Superstructure/IsHubEnabled", isHubEnabled);
  }

  /**
   * @return A command to deploy the intake arm.
   */
  public Command deployArm() {
    return arm.deploy();
  }

  /**
   * @return A command to run the intake spinner.
   */
  public Command intake() {
    return spinner.intake();
  }

  /**
   * @return A command to reverse the intake spinner (extake).
   */
  public Command extake() {
    return spinner.extake();
  }

  /**
   * @return A command to raise the climb mechanism.
   */
  public Command raiseClimb() {
    return climb.raise();
  }

  /**
   * @return A command to lower the climb mechanism.
   */
  public Command lowerClimb() {
    return climb.lower();
  }

  /**
   * Toggles the climb mode state. If climb mode is enabled, the intake arm is automatically
   * commanded to retract to prevent damage.
   */
  public void toggleClimbMode() {
    if (climbMode.toggle()) arm.setRetract();
  }

  /**
   * @return A trigger that is true only if the arm is safe to deploy (not climbing, and climb is
   *     stowed).
   */
  public Trigger canDeployArm() {
    return climbMode.negate().and(climb.isDown()).and(arm.isDeployed().negate());
  }

  /**
   * @return A trigger that is true if the intake is physically in a position to run.
   */
  public Trigger canRunIntake() {
    return arm.isDeployed();
  }

  /**
   * @return A trigger that is true if climb mode is active and the arm is safely retracted.
   */
  public Trigger canClimb() {
    return climbMode.and(arm.isRetracted());
  }

  /**
   * @return A trigger that is true when the robot has a target, is correctly aligned to the
   *     target's heading, and the shooter flywheel/hood are at their setpoints.
   */
  public Trigger isReadyToShoot() {
    return new Trigger(
        () ->
            currentAimingResult.hasTarget()
                && Math.abs(
                        currentAimingResult
                            .targetRotation()
                            .minus(drive.getPose().getRotation())
                            .getDegrees())
                    < DriveConstants.HEADING_ALIGNMENT_TOLERANCE.in(Degrees)
                && shooter.isShooterReady().getAsBoolean());
  }

  /**
   * @return Whether the superstructure currently has an active aiming target.
   */
  public boolean hasTarget() {
    return currentAimingResult.hasTarget();
  }

  /**
   * @return The desired robot rotation for the current aiming target.
   */
  public Rotation2d getTargetRotation() {
    return currentAimingResult.targetRotation();
  }

  /**
   * @return The WritableTrigger representing the climb mode state.
   */
  public WritableTrigger getClimbMode() {
    return climbMode;
  }

  /**
   * Returns whether or not all subsystems in the superstructure are healthy
   * 
   * @return True if all subsystems are healthy, false otherwise.
   */
  @AutoLogOutput(key = "Superstructure/IsHealthy")
  public boolean isHealthy() {
    return drive.isHealthy()
        && climb.isHealthy()
        && arm.isHealthy()
        && spinner.isHealthy()
        && shooter.isHealthy()
        && indexer.isHealthy()
        && vision.isHealthy();
  }
}
