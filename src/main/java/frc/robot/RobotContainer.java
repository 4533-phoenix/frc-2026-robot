// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOReal;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
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
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOServo;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.Aiming;
import frc.robot.util.Util;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Declares the robot's subsystems, operator interface devices, and command bindings.
 *
 * <p>Scoring logic (shooter, indexer, auto-aim) is delegated to the {@link Superstructure} state
 * machine. Bindings here express driver intent as simple boolean signals that the superstructure
 * reads each cycle.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Climb climb;
  private final Arm arm;
  private final Spinner spinner;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;

  private Supplier<Rotation2d> hubAimRotation;

  // Controllers
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private boolean climbMode = false;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * <p>Configures IO implementations based on the current mode (Real, Sim, or Replay).
   */
  public RobotContainer() {
    // Instantiate subsystems based on the running mode
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIODual(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        climb = new Climb(new ClimbIOReal());
        arm = new Arm(new ArmIOSpark());
        spinner = new Spinner(new SpinnerIOSpark());
        shooter = new Shooter(new FlywheelIOTalonFX(), new HoodIOServo());
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

    // Basic autos to just shoot the preloaded game piece from the starting pose, for both alliances
    // autoChooser.addOption(
    //     "Left Shoot Preload",
    //     Commands.sequence(
    //         Commands.runOnce(
    //             () ->
    //                 drive.setPose(
    //                     Util.flipAllianceIfNeeded(
    //                         new Pose2d(
    //                             3.536,
    //                             Constants.fieldWidth.in(Meters) - 2.437,
    //                             new Rotation2d(0))))),
    //         Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0, 0)), drive)
    //             .withTimeout(1.0),
    //         Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive),
    //         Commands.deadline(
    //             Commands.waitSeconds(15.0),
    //             Commands.startEnd(
    //                 () -> superstructure.setGoal(RobotGoal.FIRE),
    //                 () -> superstructure.setGoal(RobotGoal.IDLE)),
    //             DriveCommands.joystickDriveWithRotationPriority(
    //                 drive, () -> 0.0, () -> 0.0, superstructure::getAimingRotation))));

    // autoChooser.addOption(
    //     "Right Shoot Preload",
    //     Commands.sequence(
    //         Commands.runOnce(
    //             () ->
    //                 drive.setPose(
    //                     Util.flipAllianceIfNeeded(new Pose2d(3.536, 2.437, new Rotation2d(0))))),
    //         Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0, 0)), drive)
    //             .withTimeout(1.0),
    //         Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive),
    //         Commands.deadline(
    //             Commands.waitSeconds(15.0),
    //             Commands.startEnd(
    //                 () -> superstructure.setGoal(RobotGoal.FIRE),
    //                 () -> superstructure.setGoal(RobotGoal.IDLE)),
    //             DriveCommands.joystickDriveWithRotationPriority(
    //                 drive, () -> 0.0, () -> 0.0, superstructure::getAimingRotation))));

    // Set up autoaimer
    hubAimRotation =
        () ->
            Aiming.computeHubAiming(
                    drive.getPose().getTranslation(),
                    drive.getRotation(),
                    drive.getFieldRelativeVelocity(),
                    Constants.hubPosition,
                    ShooterConstants.shooterRobotOffset,
                    ShooterConstants.estimatedTimeOfFlight.in(Seconds),
                    false)
                .targetRotation();

    autoChooser.addOption(
        "Left Shoot Preload",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    drive.setPose(
                        Util.flipAllianceIfNeeded(
                            new Pose2d(
                                3.536,
                                Constants.fieldWidth.in(Meters) - 2.437,
                                new Rotation2d(0))))),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0, 0)), drive)
                .withTimeout(1.0),
            Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive),
            Commands.deadline(
                Commands.waitSeconds(15.0),
                shootWhenReady(),
                DriveCommands.joystickDriveWithRotationPriority(
                    drive, () -> 0.0, () -> 0.0, hubAimRotation),
                IntakeCommands.deploy(arm))));

    autoChooser.addOption(
        "Middle Shoot Preload",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    drive.setPose(
                        Util.flipAllianceIfNeeded(
                            new Pose2d(
                                3.536, Constants.fieldWidth.in(Meters) / 2.0, new Rotation2d(0))))),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0, 0)), drive)
                .withTimeout(1.0),
            Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive),
            Commands.deadline(
                Commands.waitSeconds(15.0),
                shootWhenReady(),
                DriveCommands.joystickDriveWithRotationPriority(
                    drive, () -> 0.0, () -> 0.0, hubAimRotation),
                IntakeCommands.deploy(arm))));

    autoChooser.addOption(
        "Right Shoot Preload",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    drive.setPose(
                        Util.flipAllianceIfNeeded(new Pose2d(3.536, 2.437, new Rotation2d(0))))),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0, 0)), drive)
                .withTimeout(1.0),
            Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive),
            Commands.deadline(
                Commands.waitSeconds(15.0),
                shootWhenReady(),
                DriveCommands.joystickDriveWithRotationPriority(
                    drive, () -> 0.0, () -> 0.0, hubAimRotation),
                IntakeCommands.deploy(arm))));

    // // Set up characterization routines for SysId
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Defines button-to-command mappings. Most scoring logic is handled by the {@link Superstructure}
   * state machine. Bindings here express driver intent as simple input signals.
   */
  private void configureButtonBindings() {
    // Driver
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController
        .leftTrigger()
        .and(() -> !climbMode)
        .whileTrue(
            Commands.parallel(
                DriveCommands.joystickDriveWithRotationPriority(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    hubAimRotation),
                Commands.run(
                    () -> {
                      double dist =
                          drive
                              .getPose()
                              .getTranslation()
                              .getDistance(Util.flipAllianceIfNeeded(Constants.hubPosition));
                      shooter.setTargetState(
                          ShooterKinematics.calculateShooterState(Meters.of(dist)));
                    },
                    shooter)));

    driverController
        .rightTrigger()
        .and(shooter.isShooterReady())
        .whileTrue(Commands.runEnd(indexer::run, indexer::stop, indexer));

    shooter.setDefaultCommand(Commands.run(shooter::stop, shooter));
    indexer.setDefaultCommand(Commands.run(indexer::stop, indexer));

    // Operator
    operatorController
        .leftBumper()
        .whileTrue(Commands.parallel(IntakeCommands.deploy(arm), IntakeCommands.intake(spinner)));

    operatorController
        .rightBumper()
        .whileTrue(Commands.parallel(IntakeCommands.deploy(arm), IntakeCommands.extake(spinner)));

    operatorController
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  climbMode = !climbMode;
                  if (climbMode) {
                    arm.retract();
                  }
                },
                arm));

    operatorController.povUp().and(arm.isRetracted()).whileTrue(ClimbCommands.liftUp(climb));
    operatorController.povDown().and(arm.isRetracted()).whileTrue(ClimbCommands.liftDown(climb));
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

  private Command shootWhenReady() {
    return Commands.run(
        () -> {
          double dist =
              drive
                  .getPose()
                  .getTranslation()
                  .getDistance(Util.flipAllianceIfNeeded(Constants.hubPosition));

          // Set shooter state based on distance
          shooter.setTargetState(ShooterKinematics.calculateShooterState(Meters.of(dist)));

          // Only run indexer if drivetrain is aligned (within 3 degrees) and flywheel is at speed
          boolean driveAligned =
              drive.isAlignedWithTarget(hubAimRotation.get(), Rotation2d.fromDegrees(5.0));
          if (driveAligned && shooter.isFlywheelReady().getAsBoolean()) {
            indexer.run();
          } else {
            indexer.stop();
          }
        },
        shooter,
        indexer);
  }
}
