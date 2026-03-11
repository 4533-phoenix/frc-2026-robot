// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FieldUtil;
import frc.lib.WritableTrigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSpark;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIODual;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOSpark;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSpark;
import frc.robot.subsystems.intake.arm.Arm;
import frc.robot.subsystems.intake.arm.ArmIO;
import frc.robot.subsystems.intake.arm.ArmIOSim;
import frc.robot.subsystems.intake.arm.ArmIOSpark;
import frc.robot.subsystems.intake.spinner.Spinner;
import frc.robot.subsystems.intake.spinner.SpinnerIO;
import frc.robot.subsystems.intake.spinner.SpinnerIOSim;
import frc.robot.subsystems.intake.spinner.SpinnerIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterKinematics;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSpark;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOServo;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.Aiming;
import frc.robot.util.Aiming.AimingResult;
import frc.robot.util.Util;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Declares the robot's subsystems, operator interface devices, and command bindings. */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Climb climb;
  private final Arm arm;
  private final Spinner spinner;
  private final Shooter shooter;
  private final Indexer indexer;

  // Vision
  @SuppressWarnings("unused")
  private final Vision vision;

  /** Controller for the driver. */
  public final CommandXboxController driverController = new CommandXboxController(0);

  /** Controller for the operator. */
  public final CommandXboxController operatorController = new CommandXboxController(1);

  // Suppliers
  private final Supplier<AimingResult> hubAiming;
  private final Supplier<AimingResult> lobAiming;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // State variables
  private final WritableTrigger climbMode = new WritableTrigger(false);
  private AimingResult currentAimingResult = Aiming.noTarget;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * <p>Configures IO implementations based on the current mode (Real, Sim, or Replay).
   */
  public RobotContainer() {
    // Instantiate subsystems based on the running mode
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIODual(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        climb = new Climb(new ClimbIOSpark());
        arm = new Arm(new ArmIOSpark());
        spinner = new Spinner(new SpinnerIOSpark());
        shooter = new Shooter(new FlywheelIOSpark(), new HoodIOServo());
        indexer = new Indexer(new IndexerIOSpark());
        vision = new Vision(new VisionIOPhoton(), drive);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(0),
                new ModuleIOSim(1),
                new ModuleIOSim(2),
                new ModuleIOSim(3));
        climb = new Climb(new ClimbIOSim());
        arm = new Arm(new ArmIOSim());
        spinner = new Spinner(new SpinnerIOSim());
        shooter = new Shooter(new FlywheelIOSim(), new HoodIOSim());
        indexer = new Indexer(new IndexerIOSim());
        vision = new Vision(new VisionIOSim(drive::getPose), drive);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        climb = new Climb(new ClimbIO() {});
        arm = new Arm(new ArmIO() {});
        spinner = new Spinner(new SpinnerIO() {});
        shooter = new Shooter(new FlywheelIO() {}, new HoodIO() {});
        indexer = new Indexer(new IndexerIO() {});
        vision = new Vision(new VisionIO() {}, drive);
        break;
    }

    // Set up auto routines via PathPlanner
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up aiming suppliers
    hubAiming =
        Aiming.hubAimingSupplier(
            drive::getPose,
            drive::getFieldRelativeVelocity,
            ShooterConstants.SHOOTER_ROBOT_OFFSET,
            ShooterConstants.ESTIMATED_TOF);
    lobAiming = Aiming.lobAimingSupplier(drive::getPose, ShooterConstants.SHOOTER_ROBOT_OFFSET);

    autoChooser.addOption(
        "Left Shoot Preload",
        Commands.sequence(
            arm.deploy(),
            Commands.runOnce(
                () ->
                    drive.setPose(
                        FieldUtil.flipAllianceIfNeeded(
                            new Pose2d(
                                3.536,
                                FieldUtil.FIELD_WIDTH.in(Meters) - 2.437,
                                new Rotation2d(0))))),
            shooter.run(),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0, 0)), drive)
                .withTimeout(1.0)
                .finallyDo(() -> drive.runVelocity(new ChassisSpeeds())),
            Commands.parallel(
                Commands.sequence(Commands.waitUntil(shooter.isShooterReady()), indexer.run()),
                DriveCommands.joystickDriveWithRotationPriority(
                    drive, () -> 0.0, () -> 0.0, () -> currentAimingResult.targetRotation()))));

    autoChooser.addOption(
        "Middle Shoot Preload",
        Commands.sequence(
            arm.deploy(),
            Commands.runOnce(
                () ->
                    drive.setPose(
                        FieldUtil.flipAllianceIfNeeded(
                            new Pose2d(
                                3.536,
                                FieldUtil.FIELD_WIDTH.in(Meters) / 2.0,
                                new Rotation2d(0))))),
            shooter.run(),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0, 0)), drive)
                .withTimeout(1.0)
                .finallyDo(() -> drive.runVelocity(new ChassisSpeeds())),
            Commands.parallel(
                Commands.sequence(Commands.waitUntil(shooter.isShooterReady()), indexer.run()),
                DriveCommands.joystickDriveWithRotationPriority(
                    drive, () -> 0.0, () -> 0.0, () -> currentAimingResult.targetRotation()))));

    autoChooser.addOption(
        "Right Shoot Preload",
        Commands.sequence(
            arm.deploy(),
            Commands.runOnce(
                () ->
                    drive.setPose(
                        FieldUtil.flipAllianceIfNeeded(
                            new Pose2d(3.536, 2.437, new Rotation2d(0))))),
            shooter.run(),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0, 0)), drive)
                .withTimeout(1.0)
                .finallyDo(() -> drive.runVelocity(new ChassisSpeeds())),
            Commands.parallel(
                Commands.sequence(Commands.waitUntil(shooter.isShooterReady()), indexer.run()),
                DriveCommands.joystickDriveWithRotationPriority(
                    drive, () -> 0.0, () -> 0.0, () -> currentAimingResult.targetRotation()))));

    // Configure the commands
    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Defines button-to-command mappings. Most scoring logic is handled by the {@link Superstructure}
   * state machine. Bindings here express driver intent as simple input signals.
   */
  private void configureDriverButtonBindings() {
    // When left trigger held and shooter has a target, rotate to aim at the target
    driverController
        .leftTrigger()
        .and(() -> currentAimingResult.hasTarget())
        .whileTrue(
            DriveCommands.joystickDriveWithRotationPriority(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> currentAimingResult.targetRotation()));

    // Spin up the motor if we are practicing not in match mode
    driverController
        .leftTrigger()
        .and(() -> currentAimingResult.hasTarget())
        .and(() -> !Util.isMatchMode())
        .whileTrue(shooter.runHeld());

    // When right trigger held, shooter is ready, and robot is aimed, run the indexer
    driverController
        .rightTrigger()
        .and(driverController.leftTrigger())
        .and(isRobotRotated())
        .and(shooter.isShooterReady())
        .whileTrue(indexer.run());

    new Trigger(
            () ->
                driverController.leftTrigger().getAsBoolean()
                    && isRobotRotated().getAsBoolean()
                    && shooter.isShooterReady().getAsBoolean())
        .whileTrue(
            Commands.runEnd(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5),
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  private void configureOperatorButtonBindings() {
    // If left or right bumper is pressed while the climb is down, deploy the intake arm
    operatorController
        .leftBumper()
        .or(operatorController.rightBumper())
        .and(climbMode.negate())
        .and(climb.isDown())
        .and(arm.isDeployed().negate())
        .onTrue(arm.deploy());
    operatorController.leftBumper().and(arm.isDeployed()).whileTrue(spinner.intake());
    operatorController.rightBumper().and(arm.isDeployed()).whileTrue(spinner.extake());

    // If left dpad is pressed, toggle climb mode. If climb mode is on, also retract the arm
    operatorController
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (climbMode.toggle()) arm.setRetract();
                }));

    // If climb mode is on and the arm is retracted, up dpad raises the climb and down dpad lowers
    operatorController.povUp().and(climbMode).and(arm.isRetracted()).whileTrue(climb.raise());
    operatorController.povDown().and(climbMode).and(arm.isRetracted()).whileTrue(climb.lower());

    // Rumble operator controller when climb mode is engaged
    climbMode.whileTrue(
        Commands.runEnd(
            () -> operatorController.getHID().setRumble(RumbleType.kRightRumble, 0.25),
            () -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0),
            climb));
  }

  /** Sets up the default commands for subsystems. */
  public void configureDefaultCommands() {
    // By default, drive with the joysticks
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = autoChooser.get();
    if (autoCommand == null) autoCommand = Commands.none();
    return autoCommand;
  }

  /** Periodically updates aiming logic and subsystem goals. */
  public void periodic() {
    // Get common booleans
    boolean isHubEnabled = Util.isHubEnabled();

    // Update the current aiming result based on the robot's position on the field
    Translation2d robotTranslation = drive.getPose().getTranslation();
    if (!climbMode.get()) {
      if (FieldUtil.flipAllianceIfNeeded(Constants.SHOOTING_ZONE).contains(robotTranslation)
          && (Util.isHubApproaching() || isHubEnabled)) {
        currentAimingResult = hubAiming.get();
        shooter.setShooterState(
            ShooterKinematics.calculateShooterState(currentAimingResult.distanceToTarget()));
      } else if (FieldUtil.flipAllianceIfNeeded(Constants.LOBBING_ZONE)
          .contains(robotTranslation)) {
        currentAimingResult = lobAiming.get();
        shooter.setShooterState(ShooterConstants.LOB_STATE);
      } else {
        currentAimingResult = Aiming.noTarget;
      }
    } else {
      currentAimingResult = Aiming.noTarget;
    }

    // Tell when the shooter should be on
    if (Util.isMatchMode()) {
      if (currentAimingResult.hasTarget() && !climbMode.get()) {
        shooter.setRunning();
      } else {
        shooter.setStop();
      }
    }

    // Log some useful values to AdvantageKit
    Logger.recordOutput("Container/ClimbMode", climbMode);
    Logger.recordOutput("Container/CurrentAimingResult", currentAimingResult);
    Logger.recordOutput("Container/IsHubEnabled", isHubEnabled);
  }

  private Trigger isRobotRotated() {
    return new Trigger(
        () ->
            currentAimingResult.hasTarget()
                && Math.abs(
                        currentAimingResult
                            .targetRotation()
                            .minus(drive.getPose().getRotation())
                            .getDegrees())
                    < DriveConstants.HEADING_ALIGNMENT_TOLERANCE.in(Degrees));
  }
}
