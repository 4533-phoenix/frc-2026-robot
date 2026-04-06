// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldUtil;
import frc.robot.services.control.driver.Driver;
import frc.robot.services.control.driver.DriverIO;
import frc.robot.services.control.driver.DriverProfile;
import frc.robot.services.control.driver.profiles.BasicDriverProfile;
import frc.robot.services.control.driver.profiles.DefaultDriverProfile;
import frc.robot.services.control.driver.profiles.NoAssistsDriverProfile;
import frc.robot.services.control.operator.Operator;
import frc.robot.services.control.operator.OperatorIO;
import frc.robot.services.control.operator.OperatorProfile;
import frc.robot.services.control.operator.profiles.DefaultOperatorProfile;
import frc.robot.services.vision.Vision;
import frc.robot.services.vision.VisionIO;
import frc.robot.services.vision.VisionIOSim;
import frc.robot.services.vision.VisionIOWhacknet;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSpark;
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
import frc.robot.subsystems.pdh.PDH;
import frc.robot.subsystems.pdh.PDHIO;
import frc.robot.subsystems.pdh.PDHIOReal;
import frc.robot.subsystems.pdh.PDHIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.Goal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSpark;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOServo;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.util.Util;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Declares the robot's subsystems, operator interface devices, and command bindings. */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Climber climber;
  private final Arm arm;
  private final Spinner spinner;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;

  @SuppressWarnings("unused")
  private final PDH pdh;

  // Superstructure
  private final Superstructure superstructure;

  // Control systems
  private final Driver driver;
  private final Operator operator;

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
                new GyroIODual(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        climber = new Climber(new ClimberIOSpark());
        arm = new Arm(new ArmIOSpark());
        spinner = new Spinner(new SpinnerIOSpark());
        shooter = new Shooter(new FlywheelIOSpark(), new HoodIOServo());
        indexer = new Indexer(new IndexerIOSpark());
        vision = new Vision(new VisionIOWhacknet());
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
        climber = new Climber(new ClimberIOSim());
        arm = new Arm(new ArmIOSim());
        spinner = new Spinner(new SpinnerIOSim());
        shooter = new Shooter(new FlywheelIOSim(), new HoodIOSim());
        indexer = new Indexer(new IndexerIOSim());
        vision = new Vision(new VisionIOSim(drive::getPose));
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
        climber = new Climber(new ClimberIO() {});
        arm = new Arm(new ArmIO() {});
        spinner = new Spinner(new SpinnerIO() {});
        shooter = new Shooter(new FlywheelIO() {}, new HoodIO() {});
        indexer = new Indexer(new IndexerIO() {});
        vision = new Vision(new VisionIO() {});
        pdh = new PDH(new PDHIO() {});
        break;
    }

    // Create the control choosers
    LoggedDashboardChooser<DriverProfile> driverChooser =
        new LoggedDashboardChooser<>("Driver Profile");
    driver = new Driver(new DriverIO() {}, driverChooser);
    LoggedDashboardChooser<OperatorProfile> operatorChooser =
        new LoggedDashboardChooser<>("Operator Profile");
    operator = new Operator(new OperatorIO() {}, operatorChooser);

    // Create the superstructure, which coordinates between subsystems
    superstructure = new Superstructure(drive, climber, arm, spinner, shooter, indexer);

    // Create the driver
    XboxController driverController = new XboxController(0);
    driverChooser.addDefaultOption(
        "Default",
        new DefaultDriverProfile(
            driverController,
            DriveConstants.MAX_LINEAR_VELOCITY,
            DriveConstants.MAX_LINEAR_ACCELERATION,
            DriveConstants.MAX_ANGULAR_VELOCITY,
            DriveConstants.MAX_ANGULAR_ACCELERATION,
            drive::getChassisSpeeds,
            superstructure.isReadyToShoot()));
    driverChooser.addOption(
        "Basic",
        new BasicDriverProfile(
            driverController,
            DriveConstants.MAX_LINEAR_VELOCITY,
            DriveConstants.MAX_ANGULAR_VELOCITY,
            superstructure.isReadyToShoot()));
    driverChooser.addOption(
        "No Assists",
        new NoAssistsDriverProfile(
            driverController,
            DriveConstants.MAX_LINEAR_VELOCITY,
            DriveConstants.MAX_ANGULAR_VELOCITY));

    // Create the operator
    XboxController operatorController = new XboxController(1);
    operatorChooser.addDefaultOption(
        "Default", new DefaultOperatorProfile(operatorController, superstructure.getClimbMode()));
    operatorChooser.addOption(
        "Solo Default",
        new DefaultOperatorProfile(driverController, superstructure.getClimbMode()));

    // Wire up the data flow from vision to drive and drive to vision
    drive.setIMUHighFreqConsumer(vision::broadcastTelemetry);
    vision.setVisionMeasurementConsumer(drive::addVisionMeasurement);

    // Register auto commands for PathPlanner
    registerAutoCommands();

    // Set up auto routines via PathPlanner
    autoChooser =
        new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser("None"));
    autoChooser.addDefaultOption("None", fallbackPoseResetCommand());

    // Assign auto commands to the chooser
    autoChooser.addOption(
        "Left Shoot Preload",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    drive.setPose(
                        FieldUtil.flipAllianceIfNeeded(
                            new Pose2d(
                                3.536,
                                FieldUtil.FIELD_WIDTH.in(Meters) - 2.437,
                                new Rotation2d(0))))),
            arm.deploy(),
            shooter.run(),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0, 0)), drive)
                .withTimeout(1.0)
                .finallyDo(() -> drive.runVelocity(new ChassisSpeeds())),
            Commands.parallel(
                Commands.sequence(
                        Commands.waitUntil(superstructure.isReadyToShoot()),
                        superstructure.feedBalls().onlyWhile(superstructure.isReadyToShoot()))
                    .repeatedly(),
                drive.headingAim(superstructure::getTargetRotation))));

    autoChooser.addOption(
        "Middle Shoot Preload",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    drive.setPose(
                        FieldUtil.flipAllianceIfNeeded(
                            new Pose2d(
                                3.536,
                                FieldUtil.FIELD_WIDTH.in(Meters) / 2.0,
                                new Rotation2d(0))))),
            arm.deploy(),
            shooter.run(),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0, 0)), drive)
                .withTimeout(1.0)
                .finallyDo(() -> drive.runVelocity(new ChassisSpeeds())),
            Commands.parallel(
                Commands.sequence(
                        Commands.waitUntil(superstructure.isReadyToShoot()),
                        superstructure.feedBalls().onlyWhile(superstructure.isReadyToShoot()))
                    .repeatedly(),
                drive.headingAim(superstructure::getTargetRotation))));

    autoChooser.addOption(
        "Right Shoot Preload",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    drive.setPose(
                        FieldUtil.flipAllianceIfNeeded(
                            new Pose2d(3.536, 2.437, new Rotation2d(0))))),
            arm.deploy(),
            shooter.run(),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0, 0)), drive)
                .withTimeout(1.0)
                .finallyDo(() -> drive.runVelocity(new ChassisSpeeds())),
            Commands.parallel(
                Commands.sequence(
                        Commands.waitUntil(superstructure.isReadyToShoot()),
                        superstructure.feedBalls().onlyWhile(superstructure.isReadyToShoot()))
                    .repeatedly(),
                drive.headingAim(superstructure::getTargetRotation))));

    // Configure the commands
    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configureDefaultCommands();
  }

  /** Registers named commands that can be triggered from PP. */
  private void registerAutoCommands() {
    // General bot controls
    NamedCommands.registerCommand("Deploy Arm", arm.deploy());
    NamedCommands.registerCommand("Retract Arm", arm.retract());
    NamedCommands.registerCommand("Start Intake", spinner.startIntake());
    NamedCommands.registerCommand("Stop Intake", spinner.stop());
    NamedCommands.registerCommand("Climber Up", climber.raise());
    NamedCommands.registerCommand("Climber Down", climber.lower());
    NamedCommands.registerCommand("Climber Stop", climber.stop());

    // Control the shooting pipeline
    NamedCommands.registerCommand("Shooter Tracking", shooter.run());
    NamedCommands.registerCommand("Shooter Stop", shooter.stop());
    NamedCommands.registerCommand(
        "Enable Drive Aim",
        Commands.runOnce(
            () -> drive.setHeadingOverrideSupplier(superstructure::getTargetRotation)));
    NamedCommands.registerCommand(
        "Disable Drive Aim", Commands.runOnce(() -> drive.setHeadingOverrideSupplier(null)));

    // Start shooting
    NamedCommands.registerCommand(
        "Shoot When Ready",
        Commands.sequence(
                Commands.waitUntil(superstructure.isReadyToShoot()),
                superstructure.feedBalls().onlyWhile(superstructure.isReadyToShoot()))
            .repeatedly());
    NamedCommands.registerCommand(
        "Hold and Shoot",
        Commands.parallel(
            Commands.sequence(
                    Commands.waitUntil(superstructure.isReadyToShoot()),
                    superstructure.feedBalls().onlyWhile(superstructure.isReadyToShoot()))
                .repeatedly(),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds()), drive)));
  }

  /**
   * Defines button-to-command mappings. Most scoring logic is handled by the {@link Superstructure}
   * state machine. Bindings here express driver intent as simple input signals.
   */
  private void configureDriverButtonBindings() {
    // When left trigger held and shooter has a target, rotate to aim at the target
    driver
        .wantsAim()
        .and(superstructure.hasTarget())
        .whileTrue(drive.headingAim(superstructure::getTargetRotation));

    // Spin up the motor if we are practicing not in match mode
    driver
        .wantsAim()
        .and(superstructure.hasTarget())
        .and(() -> !Util.isMatchMode())
        .whileTrue(shooter.runHeld());

    // When right trigger held, shooter is ready, and robot is aimed, run the indexer and oscillate
    // arm
    driver
        .wantsShoot()
        .and(driver.wantsAim())
        .and(superstructure.isReadyToShoot())
        .whileTrue(superstructure.feedBalls());

    // When B is pressed, reset the current pose to the alliance origin facing away
    driver
        .wantsReset()
        .and(() -> !Util.isMatchMode() || Util.isMatchModeOverridden())
        .onTrue(
            Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(
                            drive.getPose().getTranslation(),
                            FieldUtil.flipAllianceIfNeeded(Rotation2d.kZero)))));
  }

  private void configureOperatorButtonBindings() {
    // If left or right bumper is pressed while the climb is down, deploy the intake arm
    operator
        .wantsArmDeployment()
        .and(superstructure.canDeployArm())
        .onTrue(superstructure.deployArm());
    operator.wantsIntake().and(superstructure.canRunIntake()).whileTrue(superstructure.intake());
    operator.wantsExtake().and(superstructure.canRunIntake()).whileTrue(superstructure.extake());

    // Retract intake
    operator.wantsArmRetraction().onTrue(superstructure.retractArm());

    // Toggle climb mode. If climb mode is on, also retract the arm
    operator.wantsClimb().onTrue(Commands.runOnce(superstructure::toggleClimbMode));

    // If climb mode is on and the arm is retracted
    operator.wantsClimberUp().and(superstructure.canClimb()).whileTrue(superstructure.raiseClimb());
    operator
        .wantsClimberDown()
        .and(superstructure.canClimb())
        .whileTrue(superstructure.lowerClimb());
  }

  /** Sets up the default commands for subsystems. */
  public void configureDefaultCommands() {
    // By default, drive with the joysticks and ensure the goal is DRIVE
    drive.setDefaultCommand(driver.createDriveCommand(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = autoChooser.get();
    if (autoCommand == null) {
      return fallbackPoseResetCommand();
    }
    return autoCommand
        .asProxy()
        .finallyDo(
            () -> {
              drive.setHeadingOverrideSupplier(null);
              shooter.setGoal(Goal.STOP);
            });
  }

  private Command fallbackPoseResetCommand() {
    return Commands.runOnce(
            () ->
                drive.setPose(
                    FieldUtil.flipAllianceIfNeeded(new Pose2d(3.5, 2.4, new Rotation2d(0)))))
        .withName("Fallback Pose Reset");
  }
}
