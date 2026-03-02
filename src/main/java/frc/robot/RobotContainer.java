// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOServo;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOChalkydri;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.HardwareConfigManager;
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
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;

  // Superstructure coordinator
  private final Superstructure superstructure;

  // Tracks whether the driver has initiated a climb (latching)
  private boolean climbRequested = false;

  // Controllers
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandGenericHID operatorController = new CommandGenericHID(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
        intake = new Intake(new IntakeIOReal());
        shooter = new Shooter(new FlywheelIOTalonFX(), new HoodIOServo());
        indexer = new Indexer(new IndexerIOSpark());
        vision = new Vision(new VisionIOChalkydri(), drive);
        HardwareConfigManager.startConfigThread();
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
        intake = new Intake(new IntakeIOSim());
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
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new FlywheelIO() {}, new HoodIO() {});
        indexer = new Indexer(new IndexerIO() {});
        vision = new Vision(new VisionIO() {}, drive);
        break;
    }

    // Set up auto routines via PathPlanner
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up characterization routines for SysId
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // Build the superstructure coordinator after subsystems and bindings are initialized
    superstructure =
        new Superstructure(
            drive,
            shooter,
            indexer,
            intake,
            driverController,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> driverController.getLeftTriggerAxis() > 0.5,
            () -> driverController.getRightTriggerAxis() > 0.5,
            () -> climbRequested);

    // The superstructure periodic command owns the shooter and indexer subsystems, replacing
    // the old per-subsystem default commands for those mechanisms.
    shooter.setDefaultCommand(superstructure.getPeriodicCommand());

    // Intake default command is driven by the superstructure state (deployed normally,
    // retracted during climbing).
    intake.setDefaultCommand(superstructure.getIntakeDefaultCommand());

    // Schedule auto-aim drive whenever the superstructure requests it
    new Trigger(superstructure::wantsAutoAim).whileTrue(superstructure.getAutoAimDriveCommand());

    // Endgame rumble alert when the match enters its last 30 seconds
    new Trigger(superstructure::isEndgame)
        .onTrue(
            Superstructure.rumbleCommand(
                driverController, GenericHID.RumbleType.kBothRumble, 1.0, 0.5));
  }

  /**
   * Defines button-to-command mappings. Most scoring logic is handled by the {@link Superstructure}
   * state machine. Bindings here express driver intent as simple input signals.
   */
  private void configureButtonBindings() {
    // Normal field-relative drive (default command, overridden by auto-aim when active)
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Intake spinner overlay (orthogonal to the state machine)
    driverController.leftBumper().whileTrue(IntakeCommands.runSpinnersIn(intake));
    driverController.rightBumper().whileTrue(IntakeCommands.runSpinnersOut(intake));

    // Reset gyro heading
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Climb controls (latching state transition)
    driverController
        .povUp()
        .onTrue(Commands.runOnce(() -> climbRequested = true).andThen(ClimbCommands.liftUp(climb)));
    driverController
        .povDown()
        .onTrue(
            Commands.runOnce(() -> climbRequested = true).andThen(ClimbCommands.liftDown(climb)));
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
}
