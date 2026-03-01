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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
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
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Climb climb;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;

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
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Get common triggers from the superstructure
    Trigger outpostEnabled = Superstructure.isOutpostEnabled();
    Trigger readyToFire = Superstructure.isReadyToFire(drive, shooter);
    Trigger endgame = Superstructure.isEndgame();

    // Normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDriveAtAngle(
    //         drive,
    //         () -> -driverController.getLeftY(),
    //         () -> -driverController.getLeftX(),
    //         () -> new Rotation2d(-driverController.getRightX(), -driverController.getRightY())));

    // Intake remains retracted by default
    intake.setDefaultCommand(IntakeCommands.holdRetracted(intake));

    // Shooter and indexer stop by default
    shooter.setDefaultCommand(ShooterCommands.stopShooter(shooter));
    indexer.setDefaultCommand(IndexerCommands.stopIndexer(indexer));

    // Deploy intake and spin while Left Bumper is held
    driverController.leftBumper().whileTrue(IntakeCommands.intake(intake));
    driverController.rightBumper().whileTrue(IntakeCommands.extake(intake));

    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Auto-aim while holding Left Trigger and in shooting zone
    driverController
        .leftTrigger()
        .and(Superstructure.isInShootingZone(drive))
        .whileTrue(Superstructure.getAutoAimCommand(drive, shooter, driverController));
    // driverController.leftTrigger().whileTrue(Superstructure.getShooterAimCommand(drive,
    // shooter));

    // When we press up or down dpad
    driverController.povUp().onTrue(ClimbCommands.liftUp(climb));
    driverController.povDown().onTrue(ClimbCommands.liftDown(climb));

    // Fire game piece while holding Right Trigger, ready to fire, and outpost enabled
    driverController
        .rightTrigger()
        .and(readyToFire)
        .and(outpostEnabled)
        .whileTrue(IndexerCommands.runIndexer(indexer));

    // Rumble when outpost becomes enabled
    outpostEnabled.onTrue(
        Superstructure.rumbleCommand(driverController, RumbleType.kLeftRumble, 0.5, 0.2)
            .andThen(Commands.waitSeconds(0.1))
            .andThen(
                Superstructure.rumbleCommand(driverController, RumbleType.kLeftRumble, 0.5, 0.2)));

    // Rumble when shooter is ready
    readyToFire.whileTrue(
        Commands.startEnd(
            () -> driverController.setRumble(RumbleType.kRightRumble, 0.4),
            () -> driverController.setRumble(RumbleType.kRightRumble, 0.0)));

    // Rumble hard during endgame
    endgame.onTrue(
        Superstructure.rumbleCommand(driverController, RumbleType.kBothRumble, 1.0, 1.5));
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
