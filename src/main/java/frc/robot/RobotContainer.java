// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.FieldUtil;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIONavX;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOSpark;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.arm.Arm;
import frc.robot.subsystems.intake.arm.ArmIO;
import frc.robot.subsystems.intake.arm.ArmIOSim;
import frc.robot.subsystems.intake.spinner.Spinner;
import frc.robot.subsystems.intake.spinner.SpinnerIO;
import frc.robot.subsystems.intake.spinner.SpinnerIOSim;
import frc.robot.subsystems.pdh.PDH;
import frc.robot.subsystems.pdh.PDHIO;
import frc.robot.subsystems.pdh.PDHIOReal;
import frc.robot.subsystems.pdh.PDHIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOChalkydri;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.Util;
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
  private final Vision vision;
  private final PDH pdh;

  // Superstructure
  private final Superstructure superstructure;

  /** Controller for the driver. */
  public final CommandXboxController driverController = new CommandXboxController(0);

  /** Controller for the operator. */
  public final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                new GyroIONavX(NavXComType.kMXP_SPI),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        climb = new Climb(new ClimbIO() {});
        arm = new Arm(new ArmIO() {});
        spinner = new Spinner(new SpinnerIO() {});
        shooter = new Shooter(new FlywheelIO() {}, new HoodIO() {});
        indexer = new Indexer(new IndexerIO() {});
        vision = new Vision(new VisionIOChalkydri(), drive);
        pdh = new PDH(new PDHIOReal());
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
        pdh = new PDH(new PDHIOSim());
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
        pdh = new PDH(new PDHIO() {});
        break;
    }

    // Create the superstructure, which coordinates between subsystems
    superstructure = new Superstructure(drive, climb, arm, spinner, shooter, indexer, vision, pdh);

    // Set up auto routines via PathPlanner
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Assign auto commands to the chooser
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
                    drive, () -> 0.0, () -> 0.0, superstructure::getTargetRotation))));

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
                    drive, () -> 0.0, () -> 0.0, superstructure::getTargetRotation))));

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
                    drive, () -> 0.0, () -> 0.0, superstructure::getTargetRotation))));

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
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
        .and(superstructure::hasTarget)
        .whileTrue(
            DriveCommands.joystickDriveWithRotationPriority(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                superstructure::getTargetRotation));

    // Spin up the motor if we are practicing not in match mode
    driverController
        .leftTrigger()
        .and(superstructure::hasTarget)
        .and(() -> !Util.isMatchMode())
        .whileTrue(shooter.runHeld());

    // When right trigger held, shooter is ready, and robot is aimed, run the indexer
    driverController
        .rightTrigger()
        .and(driverController.leftTrigger())
        .and(superstructure.isReadyToShoot())
        .whileTrue(indexer.run());

    driverController
        .leftTrigger()
        .and(superstructure.isReadyToShoot())
        .whileTrue(
            Commands.runEnd(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5),
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));

    // Zero the gyro when the Y button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  private void configureOperatorButtonBindings() {
    // If left or right bumper is pressed while the climb is down, deploy the intake arm
    operatorController
        .leftBumper()
        .or(operatorController.rightBumper())
        .and(superstructure.canDeployArm())
        .onTrue(superstructure.deployArm());
    operatorController
        .leftBumper()
        .and(superstructure.canRunIntake())
        .whileTrue(superstructure.intake());
    operatorController
        .rightBumper()
        .and(superstructure.canRunIntake())
        .whileTrue(superstructure.extake());

    // If left dpad is pressed, toggle climb mode. If climb mode is on, also retract the arm
    operatorController.povLeft().onTrue(Commands.runOnce(superstructure::toggleClimbMode));

    // If climb mode is on and the arm is retracted, up dpad raises the climb and down dpad lowers
    operatorController
        .povUp()
        .and(superstructure.canClimb())
        .whileTrue(superstructure.raiseClimb());
    operatorController
        .povDown()
        .and(superstructure.canClimb())
        .whileTrue(superstructure.lowerClimb());

    // Rumble operator controller when climb mode is engaged
    superstructure
        .getClimbMode()
        .whileTrue(
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
}
