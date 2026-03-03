// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SuperstructureStates.RobotGoal;
import frc.robot.SuperstructureStates.RobotState;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterKinematics;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

/**
 * Central coordinator that evaluates field context, driver intent, and subsystem state to determine
 * the robot's behavior each cycle.
 *
 * <p>Runs a state machine over {@link RobotState} to control the shooter, indexer, and drive
 * rotation. Intake spinners are handled as an independent overlay controlled directly by driver
 * bumpers and are not part of the state machine.
 *
 * <p>This class is not a subsystem. Its {@link #periodic()} method is invoked every cycle via a
 * command scheduled on the shooter and indexer subsystems so that the state machine has exclusive
 * ownership of those mechanisms.
 */
public class Superstructure {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Climb climb;

  // State machine
  private RobotGoal currentGoal = RobotGoal.IDLE;
  private RobotState currentState = RobotState.IDLE;
  private RobotState previousState = RobotState.IDLE;
  private boolean isLobbing = false;
  private boolean lobTargetIsLeft = true;
  private boolean readyToFire = false;
  private AimingResult latestAiming = new AimingResult(Rotation2d.kZero, Meters.of(0.0));

  // Ready-to-fire must be stable for a short period before allowing the shot
  private final Debouncer readyToFireDebouncer =
      new Debouncer(0.06, Debouncer.DebounceType.kRising);

  /** Cached result of the aiming pipeline. */
  private record AimingResult(Rotation2d targetRotation, Distance distanceToTarget) {}

  /**
   * Creates a new Superstructure coordinator.
   *
   * @param drive The drive subsystem.
   * @param shooter The shooter subsystem.
   * @param indexer The indexer subsystem.
   * @param intake The intake subsystem.
   * @param climb The climb subsystem (used for auto-retract on climb exit).
   */
  public Superstructure(Drive drive, Shooter shooter, Indexer indexer, Intake intake, Climb climb) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.climb = climb;

    // Default to off so the robot does nothing automatic until explicitly enabled
    SmartDashboard.putBoolean("Match Mode", false);
  }

  /** Gets the current requested goal of the superstructure. */
  public RobotGoal getGoal() {
    return currentGoal;
  }

  /**
   * Sets the goal of the superstructure. If the current goal is CLIMB, it will ignore new goals to
   * prevent accidental overrides. Use {@link #forceGoal(Goal)} to bypass this.
   */
  public void setGoal(RobotGoal goal) {
    if (goal != RobotGoal.CLIMB && currentGoal == RobotGoal.CLIMB) {
      return;
    }
    this.currentGoal = goal;
  }

  /** Forcibly sets the superstructure goal, overriding any protections (e.g. escaping CLIMB). */
  public void forceGoal(RobotGoal goal) {
    this.currentGoal = goal;
  }

  /**
   * Evaluates the current state, applies outputs to all subsystems, and handles rumble feedback.
   * Must be called every cycle.
   */
  public void periodic() {
    previousState = currentState;
    currentState = evaluateState();

    // Compute aiming whenever the shooter is actively tracking the target
    if (currentState == RobotState.TRACKING
        || currentState == RobotState.AIMING
        || currentState == RobotState.FIRING) {
      latestAiming =
          isLobbing ? computeLobAiming(true) : computeAiming(Constants.hubPosition, true);
    }

    applyState(currentState);

    // Auto-retract the climber when exiting CLIMBING state
    if (previousState == RobotState.CLIMBING && currentState != RobotState.CLIMBING) {
      CommandScheduler.getInstance().schedule(ClimbCommands.liftDown(climb));
    }

    Logger.recordOutput("Superstructure/State", currentState.toString());
    Logger.recordOutput("Superstructure/PreviousState", previousState.toString());
    Logger.recordOutput("Superstructure/HubEnabled", Util.isHubEnabled());
    Logger.recordOutput("Superstructure/IsLobbing", isLobbing);
    Logger.recordOutput("Superstructure/MatchMode", Util.isMatchMode());

    // Evaluate and log all ready-to-shoot signals once per cycle
    updateReadyToFire();
  }

  /**
   * Determines which {@link RobotState} the robot should be in based on field context, driver
   * input, and sensor state.
   *
   * <p>When match mode is disabled (the default), the robot stays IDLE until the driver explicitly
   * holds LT. This prevents unexpected flywheel spin-up during practice and pit testing. Lobbing
   * and hub scoring still work normally via LT/RT, but automatic WARMING and TRACKING are
   * suppressed.
   */
  private RobotState evaluateState() {
    // Climbing is the active goal
    if (currentGoal == RobotGoal.CLIMB) {
      isLobbing = false;
      return RobotState.CLIMBING;
    }

    if (DriverStation.isDisabled()) {
      isLobbing = false;
      return RobotState.IDLE;
    }

    boolean matchMode = Util.isMatchMode();
    boolean aiming = (currentGoal == RobotGoal.AIM || currentGoal == RobotGoal.FIRE);
    boolean firing = (currentGoal == RobotGoal.FIRE);

    boolean canLob = isInLobbingZone() && (matchMode || aiming);
    boolean canShootHub = Util.isHubEnabled() && isInShootingZone() && (matchMode || aiming);

    if (canLob || canShootHub) {
      isLobbing = canLob;
      if (firing && isReadyToFire()) {
        return RobotState.FIRING;
      }
      return aiming ? RobotState.AIMING : RobotState.TRACKING;
    }

    isLobbing = false;

    // Pre-spin the flywheel when the hub is enabled or about to enable (match mode only)
    if (matchMode && (Util.isHubEnabled() || Util.isHubApproaching())) {
      return RobotState.WARMING;
    }

    return RobotState.IDLE;
  }

  /** Sends the appropriate commands to each subsystem based on the evaluated state. */
  private void applyState(RobotState state) {
    switch (state) {
      case IDLE:
      case CLIMBING:
        shooter.stop();
        indexer.stop();
        break;

      case WARMING:
        shooter.setIdleSpeed();
        indexer.stop();
        break;

      case TRACKING:
      case AIMING:
        shooter.setTargetState(getTargetShooterState());
        indexer.stop();
        break;

      case FIRING:
        shooter.setTargetState(getTargetShooterState());
        indexer.run();
        break;
    }
  }

  /** Returns the shooter state for the current scoring mode (lobbing or hub). */
  private ShooterState getTargetShooterState() {
    if (isLobbing) {
      return new ShooterState(lobFlywheelSpeed, lobHoodAngle);
    }
    return ShooterKinematics.calculateShooterState(latestAiming.distanceToTarget());
  }

  /** Returns true if the robot's current position is within the designated shooting zone. */
  private boolean isInShootingZone() {
    Rectangle2d shootingZone = Util.flipAllianceIfNeeded(Constants.shootingZone);
    return shootingZone.contains(drive.getPose().getTranslation());
  }

  /** Returns true if the robot's current position is within the designated lobbing zone. */
  private boolean isInLobbingZone() {
    Rectangle2d lobbingZone = Util.flipAllianceIfNeeded(Constants.lobbingZone);
    return lobbingZone.contains(drive.getPose().getTranslation());
  }

  /**
   * Returns true if the drive heading is aligned with the target and the shooter is at the
   * commanded speed and hood angle. Logs all contributing signals under {@code ReadyToShoot/}.
   */
  public boolean isReadyToFire() {
    return readyToFire;
  }

  /** Evaluates if the system is ready to fire, updates debouncer, and logs the result. */
  private void updateReadyToFire() {
    Rotation2d targetAngle = latestAiming.targetRotation();
    double headingErrorDeg = drive.getRotation().minus(targetAngle).getDegrees();
    boolean driveReady =
        Math.abs(headingErrorDeg) < DriveConstants.headingAlignmentTolerance.in(Degrees);

    boolean flywheelReady = shooter.isFlywheelReady();
    boolean hoodReady = shooter.isHoodReady();
    boolean isShooting = shooter.isShooting();
    boolean shooterReady = flywheelReady && hoodReady && isShooting;

    boolean ready = driveReady && shooterReady;
    readyToFire = readyToFireDebouncer.calculate(ready);

    Logger.recordOutput("Superstructure/ReadyToFire/Ready", readyToFire);
    Logger.recordOutput("Superstructure/ReadyToFire/RawReady", ready);
    Logger.recordOutput("Superstructure/ReadyToFire/DriveAligned", driveReady);
    Logger.recordOutput("Superstructure/ReadyToFire/HeadingErrorDeg", headingErrorDeg);
    Logger.recordOutput("Superstructure/ReadyToFire/FlywheelReady", flywheelReady);
    Logger.recordOutput(
        "Superstructure/ReadyToFire/FlywheelErrorRadPerSec", shooter.getFlywheelErrorRadPerSec());
    Logger.recordOutput("Superstructure/ReadyToFire/HoodReady", hoodReady);
    Logger.recordOutput("Superstructure/ReadyToFire/IsShooting", isShooting);
    Logger.recordOutput("Superstructure/ReadyToFire/ShooterReady", shooterReady);
  }

  /**
   * Returns the field-space position of the shooter mechanism, accounting for the physical offset
   * from the robot center.
   */
  private Translation2d getShooterFieldPosition() {
    Pose2d robotPose = drive.getPose();
    return robotPose.getTranslation().plus(shooterRobotOffset.rotateBy(robotPose.getRotation()));
  }

  /**
   * Returns the predicted field-space position of the shooter after the robot rotates to the given
   * heading. The robot center stays the same; only the offset rotates.
   */
  private Translation2d getShooterFieldPositionAtHeading(Rotation2d heading) {
    return drive.getPose().getTranslation().plus(shooterRobotOffset.rotateBy(heading));
  }

  /**
   * Computes all aiming outputs using a two-pass approach. The first pass estimates the target
   * heading from the current shooter position. The second pass predicts where the shooter will be
   * once the robot rotates to that heading and recomputes the final rotation and distance. This
   * gives the shooter kinematics a more accurate distance to work with during TRACKING before the
   * robot has actually turned.
   *
   * @param targetPosition The blue-alliance target position (flipped automatically).
   * @param log Whether to publish outputs to AdvantageKit for visualization.
   */
  private AimingResult computeAiming(Translation2d targetPosition, boolean log) {
    Translation2d targetTranslation = Util.flipAllianceIfNeeded(targetPosition);
    Translation2d fieldVelocity = drive.getFieldRelativeVelocity();

    // Estimate rotation from current shooter position
    Translation2d currentShooterPos = getShooterFieldPosition();
    Translation2d initialLead =
        Util.calculateClampedLead(
            currentShooterPos, targetTranslation, fieldVelocity, estimatedTimeOfFlight.in(Seconds));
    Translation2d initialVirtualTarget = targetTranslation.minus(initialLead);
    Rotation2d estimatedRotation = initialVirtualTarget.minus(currentShooterPos).getAngle();

    // Predict shooter position at the estimated rotation and recompute
    Translation2d predictedShooterPos = getShooterFieldPositionAtHeading(estimatedRotation);
    Translation2d clampedLead =
        Util.calculateClampedLead(
            predictedShooterPos,
            targetTranslation,
            fieldVelocity,
            estimatedTimeOfFlight.in(Seconds));
    Translation2d virtualTarget = targetTranslation.minus(clampedLead);

    Translation2d shooterToTarget = virtualTarget.minus(predictedShooterPos);
    Rotation2d targetRotation = shooterToTarget.getAngle();
    double distanceMeters = shooterToTarget.getNorm();

    if (log) {
      Logger.recordOutput("Aiming/VirtualTarget", new Pose2d(virtualTarget, Rotation2d.kZero));
      Logger.recordOutput("Aiming/StaticTarget", new Pose2d(targetTranslation, Rotation2d.kZero));
      Logger.recordOutput(
          "Aiming/ShooterPosition", new Pose2d(currentShooterPos, drive.getRotation()));
      Logger.recordOutput(
          "Aiming/PredictedShooterPosition", new Pose2d(predictedShooterPos, estimatedRotation));
      Logger.recordOutput("Aiming/TargetRotation", targetRotation.getDegrees());
      Logger.recordOutput("Aiming/DistanceToTarget", distanceMeters);
    }

    return new AimingResult(targetRotation, Meters.of(distanceMeters));
  }

  /**
   * Computes aiming for lobbed shots. Each lobbing target is a 2m vertical line segment on the
   * field. The method picks the nearest line with hysteresis, then aims at the closest point on
   * that line. Uses a two-pass approach to predict the shooter position after rotation. Lead
   * compensation is skipped because lobbed shots are high-arc and imprecise.
   *
   * @param log Whether to publish outputs to AdvantageKit for visualization.
   */
  private AimingResult computeLobAiming(boolean log) {
    Translation2d leftCenter = Util.flipAllianceIfNeeded(Constants.lobbingTargetLeftCenter);
    Translation2d rightCenter = Util.flipAllianceIfNeeded(Constants.lobbingTargetRightCenter);
    double halfLen = Constants.lobbingTargetHalfLength.in(Meters);
    Translation2d currentShooterPos = getShooterFieldPosition();

    // Find closest point on each line segment to the current shooter position
    Translation2d closestLeft = Util.closestPointOnLobLine(currentShooterPos, leftCenter, halfLen);
    Translation2d closestRight =
        Util.closestPointOnLobLine(currentShooterPos, rightCenter, halfLen);

    double distLeft = currentShooterPos.getDistance(closestLeft);
    double distRight = currentShooterPos.getDistance(closestRight);

    // Only switch targets when the other is at least 0.5m closer
    double hysteresis = 0.5;
    if (lobTargetIsLeft && distRight < distLeft - hysteresis) {
      lobTargetIsLeft = false;
    } else if (!lobTargetIsLeft && distLeft < distRight - hysteresis) {
      lobTargetIsLeft = true;
    }

    Translation2d target = lobTargetIsLeft ? closestLeft : closestRight;

    // Estimate rotation from current shooter position
    Rotation2d estimatedRotation = target.minus(currentShooterPos).getAngle();

    // Predict shooter position at the estimated rotation, re-find closest point
    Translation2d predictedShooterPos = getShooterFieldPositionAtHeading(estimatedRotation);
    Translation2d lineCenter = lobTargetIsLeft ? leftCenter : rightCenter;
    target = Util.closestPointOnLobLine(predictedShooterPos, lineCenter, halfLen);

    Translation2d shooterToTarget = target.minus(predictedShooterPos);
    Rotation2d targetRotation = shooterToTarget.getAngle();
    double distanceMeters = shooterToTarget.getNorm();

    if (log) {
      Logger.recordOutput("Aiming/LobTarget", new Pose2d(target, Rotation2d.kZero));
      Logger.recordOutput(
          "Aiming/ShooterPosition", new Pose2d(currentShooterPos, drive.getRotation()));
      Logger.recordOutput(
          "Aiming/PredictedShooterPosition", new Pose2d(predictedShooterPos, estimatedRotation));
      Logger.recordOutput("Aiming/TargetRotation", targetRotation.getDegrees());
      Logger.recordOutput("Aiming/DistanceToTarget", distanceMeters);
    }

    return new AimingResult(targetRotation, Meters.of(distanceMeters));
  }

  /** Returns the current superstructure state. */
  public RobotState getState() {
    return currentState;
  }

  /** Returns true if the robot is in a state where auto-aim drive should be active. */
  public boolean wantsAutoAim() {
    return currentState == RobotState.AIMING || currentState == RobotState.FIRING;
  }

  /** Returns the current aimed rotation for the drive to snap to. */
  public Rotation2d getAimingRotation() {
    return latestAiming.targetRotation();
  }

  /**
   * Returns an intake default command that manages arm position based on state. During CLIMBING the
   * arm is always retracted. Otherwise, if the arm is already deployed (checked via the absolute
   * encoder) it is held deployed; if not yet deployed it is held retracted.
   */
  public Command getIntakeDefaultCommand() {
    return Commands.either(
        IntakeCommands.holdRetracted(intake),
        Commands.either(
            IntakeCommands.holdDeployed(intake),
            IntakeCommands.holdRetracted(intake),
            intake::armDeployed),
        () -> currentState == RobotState.CLIMBING);
  }

  /**
   * Returns a command that runs the superstructure periodic loop. This command requires the shooter
   * and indexer subsystems, giving the state machine exclusive control over those mechanisms. The
   * command runs even while disabled so the state machine stays coherent and logs are accurate.
   */
  public Command getPeriodicCommand() {
    return Commands.run(this::periodic, shooter, indexer).ignoringDisable(true);
  }
}
