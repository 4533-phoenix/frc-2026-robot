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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterKinematics;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.util.Util;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
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

  // Driver intent suppliers (wired from RobotContainer)
  private final BooleanSupplier aimSupplier;
  private final BooleanSupplier fireSupplier;
  private final BooleanSupplier climbingSupplier;

  // State machine
  private RobotState currentState = RobotState.IDLE;
  private RobotState previousState = RobotState.IDLE;
  private boolean isLobbing = false;
  private boolean lobTargetIsLeft = true;
  private AimingResult latestAiming = new AimingResult(Rotation2d.kZero, Meters.of(0.0));

  // Pre-built auto-aim drive command, scheduled and cancelled by RobotContainer
  private final Command autoAimDriveCommand;

  // Rumble
  private final CommandGenericHID driverController;
  private boolean readyRumbleActive = false;
  private boolean previousOutpostEnabled = false;

  // Ready-to-fire must be stable for a short period before allowing the shot
  private final Debouncer readyToFireDebouncer =
      new Debouncer(0.06, Debouncer.DebounceType.kRising);

  // Match mode toggle (defaults to off so the robot idles until the driver acts)
  private static final String MATCH_MODE_KEY = "Match Mode";

  /** Cached result of the aiming pipeline. */
  private record AimingResult(Rotation2d targetRotation, Distance distanceToTarget) {}

  /**
   * Creates a new Superstructure coordinator.
   *
   * @param drive The drive subsystem.
   * @param shooter The shooter subsystem.
   * @param indexer The indexer subsystem.
   * @param intake The intake subsystem.
   * @param driverController The driver's controller for rumble feedback.
   * @param xSupplier Driver forward/backward input (negated from controller).
   * @param ySupplier Driver strafe input (negated from controller).
   * @param aimSupplier True when the driver wants to auto-aim (LT).
   * @param fireSupplier True when the driver wants to fire (RT).
   * @param climbingSupplier True when the driver has initiated a climb.
   */
  public Superstructure(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      CommandGenericHID driverController,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      BooleanSupplier aimSupplier,
      BooleanSupplier fireSupplier,
      BooleanSupplier climbingSupplier) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.driverController = driverController;
    this.aimSupplier = aimSupplier;
    this.fireSupplier = fireSupplier;
    this.climbingSupplier = climbingSupplier;

    autoAimDriveCommand =
        DriveCommands.joystickDriveWithRotationPriority(
            drive, xSupplier, ySupplier, () -> latestAiming.targetRotation());

    // Default to off so the robot does nothing automatic until explicitly enabled
    SmartDashboard.putBoolean(MATCH_MODE_KEY, false);
  }

  /**
   * Returns true when match mode is active. Match mode enables automatically when a match timer is
   * running (FMS or driver station practice mode). It can also be forced on via the dashboard
   * toggle for testing.
   */
  private boolean isMatchMode() {
    if (SmartDashboard.getBoolean(MATCH_MODE_KEY, false)) return true;
    double time = DriverStation.getMatchTime();
    return DriverStation.isFMSAttached() || time > 0;
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
      if (isLobbing) {
        latestAiming = computeLobAiming(true);
      } else {
        latestAiming = computeAiming(Constants.outpostPosition, true);
      }
    }

    applyState(currentState);
    handleRumble();

    Logger.recordOutput("Superstructure/State", currentState.toString());
    Logger.recordOutput("Superstructure/PreviousState", previousState.toString());
    Logger.recordOutput("Superstructure/OutpostEnabled", isOutpostEnabled());
    Logger.recordOutput("Superstructure/IsLobbing", isLobbing);
    Logger.recordOutput("Superstructure/MatchMode", isMatchMode());

    // Log all ready-to-shoot signals (writes to ReadyToShoot/ folder)
    isReadyToFire();
  }

  /**
   * Determines which {@link RobotState} the robot should be in based on field context, driver
   * input, and sensor state.
   *
   * <p>When match mode is disabled (the default), the robot stays IDLE until the driver explicitly
   * holds LT. This prevents unexpected flywheel spin-up during practice and pit testing. Lobbing
   * and outpost scoring still work normally via LT/RT, but automatic WARMING and TRACKING are
   * suppressed.
   */
  private RobotState evaluateState() {
    // Climbing is a latching state controlled exclusively by the driver
    if (climbingSupplier.getAsBoolean() || currentState == RobotState.CLIMBING) {
      isLobbing = false;
      return RobotState.CLIMBING;
    }

    if (DriverStation.isDisabled()) {
      isLobbing = false;
      return RobotState.IDLE;
    }

    boolean matchMode = isMatchMode();
    boolean inLobbingZone = isInLobbingZone();
    boolean outpostEnabled = isOutpostEnabled();
    boolean inShootingZone = isInShootingZone();
    boolean aiming = aimSupplier.getAsBoolean();
    boolean firing = fireSupplier.getAsBoolean();

    // Lobbing is always available when in the lobbing zone, regardless of outpost state.
    // Without match mode, require LT to enter the pipeline (no automatic TRACKING).
    if (inLobbingZone && (matchMode || aiming)) {
      isLobbing = true;
      if (aiming && firing && isReadyToFire()) {
        return RobotState.FIRING;
      }
      if (aiming) {
        return RobotState.AIMING;
      }
      return RobotState.TRACKING;
    }

    isLobbing = false;

    // Outpost scoring when in shooting zone with active outpost.
    // Without match mode, require LT to enter the pipeline (no automatic TRACKING).
    if (outpostEnabled && inShootingZone && (matchMode || aiming)) {
      if (aiming && firing && isReadyToFire()) {
        return RobotState.FIRING;
      }
      if (aiming) {
        return RobotState.AIMING;
      }
      return RobotState.TRACKING;
    }

    // Pre-spin the flywheel when the outpost is enabled or about to enable (match mode only)
    if (matchMode && (outpostEnabled || isOutpostApproaching())) {
      return RobotState.WARMING;
    }

    return RobotState.IDLE;
  }

  /** Sends the appropriate commands to each subsystem based on the evaluated state. */
  private void applyState(RobotState state) {
    switch (state) {
      case IDLE:
        shooter.stop();
        indexer.stop();
        break;

      case WARMING:
        shooter.setIdleSpeed();
        indexer.stop();
        break;

      case TRACKING:
        shooter.setTargetState(getTargetShooterState());
        indexer.stop();
        break;

      case AIMING:
        shooter.setTargetState(getTargetShooterState());
        indexer.stop();
        break;

      case FIRING:
        shooter.setTargetState(getTargetShooterState());
        indexer.run();
        break;

      case CLIMBING:
        shooter.stop();
        indexer.stop();
        break;
    }
  }

  /** Returns the shooter state for the current scoring mode (lobbing or outpost). */
  private ShooterState getTargetShooterState() {
    if (isLobbing) {
      return new ShooterState(lobFlywheelSpeed, lobHoodAngle);
    }
    return ShooterKinematics.calculateShooterState(latestAiming.distanceToTarget());
  }

  /** Manages controller rumble based on state transitions and current conditions. */
  private void handleRumble() {
    boolean outpostEnabled = isOutpostEnabled();

    // Pulse when the outpost first becomes enabled
    if (outpostEnabled && !previousOutpostEnabled) {
      pulseRumble(RumbleType.kLeftRumble, 0.5, 0.2);
    }
    previousOutpostEnabled = outpostEnabled;

    // Sustained right rumble while aligned and ready to fire
    boolean shouldRumbleReady =
        (currentState == RobotState.AIMING || currentState == RobotState.FIRING) && isReadyToFire();

    if (shouldRumbleReady && !readyRumbleActive) {
      driverController.setRumble(RumbleType.kRightRumble, 0.4);
      readyRumbleActive = true;
    } else if (!shouldRumbleReady && readyRumbleActive) {
      driverController.setRumble(RumbleType.kRightRumble, 0.0);
      readyRumbleActive = false;
    }
  }

  /** Schedules a brief rumble pulse. */
  private void pulseRumble(RumbleType type, double intensity, double duration) {
    CommandScheduler.getInstance()
        .schedule(
            Commands.startEnd(
                    () -> driverController.setRumble(type, intensity),
                    () -> driverController.setRumble(type, 0.0))
                .withTimeout(duration));
  }

  /** Returns true if the outpost scoring mechanism is currently active. */
  private boolean isOutpostEnabled() {
    double time = DriverStation.getMatchTime();
    boolean isTimedMatch = DriverStation.isFMSAttached() || time > 0;

    if (!isTimedMatch) {
      return DriverStation.isEnabled();
    }

    if (DriverStation.isDisabled()) return false;
    if (DriverStation.isAutonomous() || time > 130 || time <= 30) return true;

    String data = DriverStation.getGameSpecificMessage();
    var alliance = DriverStation.getAlliance();
    if (data == null || data.isEmpty() || alliance.isEmpty()) return true;

    char inactiveInShift1 = data.charAt(0);
    char myColor = (alliance.get() == DriverStation.Alliance.Red) ? 'R' : 'B';
    boolean amIInactiveInShift1 = (inactiveInShift1 == myColor);

    if (time > 105) return !amIInactiveInShift1;
    else if (time > 80) return amIInactiveInShift1;
    else if (time > 55) return !amIInactiveInShift1;
    else return amIInactiveInShift1;
  }

  /**
   * Returns true if the outpost will enable within the next 5 seconds. Used to trigger the WARMING
   * state before the outpost actually activates, giving the flywheel a thermal and inertial head
   * start.
   */
  private boolean isOutpostApproaching() {
    double time = DriverStation.getMatchTime();
    boolean isTimedMatch = DriverStation.isFMSAttached() || time > 0;
    if (!isTimedMatch || DriverStation.isDisabled()) return false;

    String data = DriverStation.getGameSpecificMessage();
    var alliance = DriverStation.getAlliance();
    if (data == null || data.isEmpty() || alliance.isEmpty()) return false;

    char inactiveInShift1 = data.charAt(0);
    char myColor = (alliance.get() == DriverStation.Alliance.Red) ? 'R' : 'B';
    boolean amIInactiveInShift1 = (inactiveInShift1 == myColor);

    // Look 5 seconds ahead (match time counts down)
    double futureTime = time - 5.0;
    if (futureTime <= 0) return false;

    boolean futureEnabled;
    if (futureTime > 130 || futureTime <= 30) {
      futureEnabled = true;
    } else if (futureTime > 105) {
      futureEnabled = !amIInactiveInShift1;
    } else if (futureTime > 80) {
      futureEnabled = amIInactiveInShift1;
    } else if (futureTime > 55) {
      futureEnabled = !amIInactiveInShift1;
    } else {
      futureEnabled = amIInactiveInShift1;
    }

    return futureEnabled && !isOutpostEnabled();
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
  private boolean isReadyToFire() {
    Rotation2d targetAngle = latestAiming.targetRotation();
    double headingErrorDeg = drive.getRotation().minus(targetAngle).getDegrees();
    boolean driveReady =
        Math.abs(headingErrorDeg) < DriveConstants.headingAlignmentTolerance.in(Degrees);

    boolean flywheelReady = shooter.isFlywheelReady();
    boolean hoodReady = shooter.isHoodReady();
    boolean isShooting = shooter.isShooting();
    boolean shooterReady = flywheelReady && hoodReady && isShooting;

    boolean ready = driveReady && shooterReady;
    boolean debouncedReady = readyToFireDebouncer.calculate(ready);

    Logger.recordOutput("Superstructure/ReadyToFire/Ready", debouncedReady);
    Logger.recordOutput("Superstructure/ReadyToFire/RawReady", ready);
    Logger.recordOutput("Superstructure/ReadyToFire/DriveAligned", driveReady);
    Logger.recordOutput("Superstructure/ReadyToFire/HeadingErrorDeg", headingErrorDeg);
    Logger.recordOutput("Superstructure/ReadyToFire/FlywheelReady", flywheelReady);
    Logger.recordOutput(
        "Superstructure/ReadyToFire/FlywheelErrorRadPerSec", shooter.getFlywheelErrorRadPerSec());
    Logger.recordOutput("Superstructure/ReadyToFire/HoodReady", hoodReady);
    Logger.recordOutput("Superstructure/ReadyToFire/IsShooting", isShooting);
    Logger.recordOutput("Superstructure/ReadyToFire/ShooterReady", shooterReady);

    return debouncedReady;
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
   * Calculates the lead offset to apply to the static target, accounting for the robot's current
   * velocity and the estimated time of flight. The lead magnitude is clamped to 50% of the
   * shooter-to-target distance so the virtual target can never overshoot past the real target.
   */
  private static Translation2d calculateClampedLead(
      Translation2d shooterPos, Translation2d targetTranslation, Translation2d fieldVelocity) {
    double tof = estimatedTimeOfFlight.in(Seconds);
    Translation2d rawLead =
        new Translation2d(fieldVelocity.getX() * tof, fieldVelocity.getY() * tof);

    double distToTarget = shooterPos.getDistance(targetTranslation);
    double maxLead = distToTarget * 0.5;
    double leadMag = rawLead.getNorm();

    if (leadMag > maxLead && leadMag > 1e-6) {
      return rawLead.times(maxLead / leadMag);
    }
    return rawLead;
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
        calculateClampedLead(currentShooterPos, targetTranslation, fieldVelocity);
    Translation2d initialVirtualTarget = targetTranslation.minus(initialLead);
    Rotation2d estimatedRotation = initialVirtualTarget.minus(currentShooterPos).getAngle();

    // Predict shooter position at the estimated rotation and recompute
    Translation2d predictedShooterPos = getShooterFieldPositionAtHeading(estimatedRotation);
    Translation2d clampedLead =
        calculateClampedLead(predictedShooterPos, targetTranslation, fieldVelocity);
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
    Translation2d closestLeft = closestPointOnLobLine(currentShooterPos, leftCenter, halfLen);
    Translation2d closestRight = closestPointOnLobLine(currentShooterPos, rightCenter, halfLen);

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

    // Pass 1: estimate rotation from current shooter position
    Rotation2d estimatedRotation = target.minus(currentShooterPos).getAngle();

    // Pass 2: predict shooter position at the estimated rotation, re-find closest point
    Translation2d predictedShooterPos = getShooterFieldPositionAtHeading(estimatedRotation);
    Translation2d lineCenter = lobTargetIsLeft ? leftCenter : rightCenter;
    target = closestPointOnLobLine(predictedShooterPos, lineCenter, halfLen);

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

  /**
   * Returns the closest point on a vertical (Y-axis) lobbing line segment to the given position.
   * The line is centered at {@code center} and extends {@code halfLen} meters in each Y direction.
   */
  private static Translation2d closestPointOnLobLine(
      Translation2d from, Translation2d center, double halfLen) {
    double clampedY = MathUtil.clamp(from.getY(), center.getY() - halfLen, center.getY() + halfLen);
    return new Translation2d(center.getX(), clampedY);
  }

  /** Returns the current superstructure state. */
  public RobotState getState() {
    return currentState;
  }

  /** Returns true if the robot is in a state where auto-aim drive should be active. */
  public boolean wantsAutoAim() {
    return currentState == RobotState.AIMING || currentState == RobotState.FIRING;
  }

  /**
   * Returns the pre-built auto-aim drive command. RobotContainer is responsible for scheduling and
   * cancelling it based on {@link #wantsAutoAim()}.
   */
  public Command getAutoAimDriveCommand() {
    return autoAimDriveCommand;
  }

  /**
   * Returns an intake default command that retracts the arm during CLIMBING and holds it deployed
   * in all other states.
   */
  public Command getIntakeDefaultCommand() {
    return Commands.either(
        IntakeCommands.holdRetracted(intake),
        IntakeCommands.holdDeployed(intake),
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

  /** Returns true when the match is in the last 30 seconds of teleop. */
  public boolean isEndgame() {
    if (DriverStation.isDisabled() || DriverStation.isAutonomous()) return false;
    double time = DriverStation.getMatchTime();
    boolean isTimedMatch = DriverStation.isFMSAttached() || time > 0;
    return isTimedMatch && time <= 30.0 && time > 0;
  }

  /**
   * Creates a command to rumble a controller for a specific duration.
   *
   * @param controller The controller to rumble.
   * @param type The type of rumble (Left or Right).
   * @param intensity The intensity of the rumble (0.0 to 1.0).
   * @param duration The duration of the rumble in seconds.
   * @return A timed rumble command.
   */
  public static Command rumbleCommand(
      CommandGenericHID controller, RumbleType type, double intensity, double duration) {
    return Commands.startEnd(
            () -> controller.setRumble(type, intensity), () -> controller.setRumble(type, 0.0))
        .withTimeout(duration);
  }
}
