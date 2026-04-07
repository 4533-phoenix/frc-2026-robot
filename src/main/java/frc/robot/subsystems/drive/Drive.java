// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.IMUState;
import frc.lib.monitor.MonitoredSubsystemBase;
import frc.lib.monitor.checkers.GyroMonitor;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.util.LocalADStarAK;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for the robot's swerve drive.
 *
 * <p>Handles kinematics, odometry estimation (incorporating gyro, modules, and vision), and
 * interfacing with PathPlanner for autonomous paths.
 */
public class Drive extends MonitoredSubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final GyroMonitor gyroHealthMonitor = new GyroMonitor();

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private double lastResetTimestamp = 0.0;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private final double maxModuleRadius;

  private final TimeInterpolatableBuffer<Rotation2d> gyroHistory =
      TimeInterpolatableBuffer.createBuffer(0.5);

  // Pre-allocate sweve module positions for odometry to avoid allocations in the main loop
  private final SwerveModulePosition[] odometryPositionsBuffer =
      new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(),
        new SwerveModulePosition(), new SwerveModulePosition()
      };

  private final SwerveModulePosition[] odometryDeltasBuffer =
      new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(),
        new SwerveModulePosition(), new SwerveModulePosition()
      };

  private final SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  private final SwerveModuleState[] currentStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private final Notifier odometryThread;
  private Consumer<IMUState> visionHighFreqConsumer;
  private final LinearFilter targetVelocityFilter = LinearFilter.singlePoleIIR(0.05, 0.02);
  private final LinearFilter fieldVelocityXFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private final LinearFilter fieldVelocityYFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private Translation2d currentFieldVelocity = new Translation2d();

  private final PIDController rotationController = new PIDController(ANGLE_KP, 0.0, ANGLE_KD);

  private Supplier<Rotation2d> headingOverrideSupplier = () -> null;
  private Supplier<Double> headingVelocityOverrideSupplier = () -> 0.0;
  private Rotation2d lastTargetHeading = null;

  private final SwerveDriveKinematics[] dynamicKinematics = new SwerveDriveKinematics[16];
  private final boolean[] isCaster = new boolean[4];
  private final boolean[] isOpportunistic = new boolean[4];
  private final boolean[] isDead = new boolean[4];
  private final SwerveModuleState[] finalStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  /**
   * Creates a new Drive subsystem.
   *
   * @param gyroIO The abstraction layer for the gyroscope.
   * @param flModuleIO The abstraction layer for the front-left module.
   * @param frModuleIO The abstraction layer for the front-right module.
   * @param blModuleIO The abstraction layer for the back-left module.
   * @param brModuleIO The abstraction layer for the back-right module.
   */
  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Try our absolute best to keep this thing driving
    for (int mask = 1; mask < 16; mask++) {
      List<Translation2d> activeTranslations = new ArrayList<>();
      for (int mod = 0; mod < 4; mod++) {
        if ((mask & (1 << mod)) != 0) {
          activeTranslations.add(MODULE_TRANSLATIONS[mod]);
        }
      }

      while (activeTranslations.size() < 2) {
        activeTranslations.add(new Translation2d());
      }

      dynamicKinematics[mask] =
          new SwerveDriveKinematics(activeTranslations.toArray(new Translation2d[0]));
    }

    double maxR = 0.0;
    for (var t : MODULE_TRANSLATIONS) {
      double r = Math.hypot(t.getX(), t.getY());
      if (r > maxR) maxR = r;
    }
    this.maxModuleRadius = maxR;

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    Notifier.setHALThreadPriority(true, 50);
    odometryThread = new Notifier(this::odometryLoop);
    odometryThread.startPeriodic(1.0 / ODOMETRY_FREQUENCY.in(Hertz));

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(7.0, 0.0, 0.0), new PIDConstants(7.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage), null, this));

    // Configure PID controller
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Math.toRadians(0.5));
  }

  /**
   * Command to maintain a specific heading using a supplier for the target angle.
   *
   * @param targetSupplier A supplier for the target heading (Rotation2d).
   * @return A command that maintains the specified heading.
   */
  public Command headingAim(Supplier<Rotation2d> targetSupplier) {
    return headingAim(targetSupplier, () -> 0.0);
  }

  /**
   * Command to maintain a specific heading using a supplier for the target angle and target angular
   * velocity.
   *
   * @param targetSupplier A supplier for the target heading (Rotation2d).
   * @param velocitySupplier A supplier for the target angular velocity (rad/s) feedforward.
   * @return A command that maintains the specified heading.
   */
  public Command headingAim(
      Supplier<Rotation2d> targetSupplier, Supplier<Double> velocitySupplier) {
    return Commands.startEnd(
        () -> setHeadingOverrideSupplier(targetSupplier, velocitySupplier),
        () -> setHeadingOverrideSupplier(null));
  }

  /**
   * Returns the current pose of the robot on the field.
   *
   * @param callback A Consumer that accepts the current IMU state.
   */
  public void setIMUHighFreqConsumer(Consumer<IMUState> callback) {
    this.visionHighFreqConsumer = callback;
  }

  private void odometryLoop() {
    double timestampSec = RobotController.getFPGATime() / 1.0e6;

    IMUState imuState = gyroIO.updateHighFreq(timestampSec);
    for (var module : modules) {
      module.updateHighFreq(timestampSec);
    }

    if (visionHighFreqConsumer != null && imuState != null) {
      visionHighFreqConsumer.accept(imuState);
    }
  }

  /**
   * Sets the supplier for the heading override.
   *
   * @param supplier A Supplier that provides the desired heading as a Rotation2d. If null, the
   *     robot will use the gyro heading for odometry and control.
   */
  public void setHeadingOverrideSupplier(Supplier<Rotation2d> supplier) {
    setHeadingOverrideSupplier(supplier, () -> 0.0);
  }

  /**
   * Sets the supplier for the heading override and angular velocity feedforward.
   *
   * @param supplier A Supplier that provides the desired heading as a Rotation2d. If null, the
   *     robot will use the gyro heading for odometry and control.
   * @param velocitySupplier A Supplier that provides the target angular velocity.
   */
  public void setHeadingOverrideSupplier(
      Supplier<Rotation2d> supplier, Supplier<Double> velocitySupplier) {
    this.headingOverrideSupplier = (supplier == null) ? (() -> null) : supplier;
    this.headingVelocityOverrideSupplier =
        (velocitySupplier == null) ? (() -> 0.0) : velocitySupplier;
    if (supplier != null) {
      targetVelocityFilter.reset();
      resetRotationController();
    }
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);

    for (int i = 0; i < gyroInputs.odometryYawTimestamps.length; i++) {
      if (gyroInputs.odometryYawTimestamps[i] > lastResetTimestamp) {
        gyroHistory.addSample(
            gyroInputs.odometryYawTimestamps[i],
            Rotation2d.fromRadians(gyroInputs.odometryYawPositions[i]));
      }
    }

    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;

    // Check bounds for module positions to determine max valid sample count
    for (int i = 0; i < 4; i++) {
      sampleCount = Math.min(sampleCount, modules[i].getOdometryPositions().length);
    }
    for (int i = 0; i < sampleCount; i++) {
      double timestamp = sampleTimestamps[i];

      if (timestamp <= lastResetTimestamp) {
        continue;
      }

      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        SwerveModulePosition sample = modules[moduleIndex].getOdometryPositions()[i];
        odometryPositionsBuffer[moduleIndex].distanceMeters = sample.distanceMeters;
        odometryPositionsBuffer[moduleIndex].angle = sample.angle;

        // Calculate deltas for kinematics fallback if needed
        odometryDeltasBuffer[moduleIndex].distanceMeters =
            sample.distanceMeters - lastModulePositions[moduleIndex].distanceMeters;
        odometryDeltasBuffer[moduleIndex].angle = sample.angle;

        lastModulePositions[moduleIndex].distanceMeters = sample.distanceMeters;
        lastModulePositions[moduleIndex].angle = sample.angle;
      }

      // Interpolate the gyro rotation at the exact timestamp of this module sample
      if (gyroInputs.connected) {
        var interpolatedRotation = gyroHistory.getSample(timestamp);
        if (interpolatedRotation.isPresent()) {
          rawGyroRotation = interpolatedRotation.get();
        }
      } else {
        Twist2d twist = kinematics.toTwist2d(odometryDeltasBuffer);
        rawGyroRotation = rawGyroRotation.plus(Rotation2d.fromRadians(twist.dtheta));
      }

      poseEstimator.updateWithTime(timestamp, rawGyroRotation, odometryPositionsBuffer);
    }

    // Update gyro fault alert
    gyroHealthMonitor.update(gyroInputs);

    ChassisSpeeds robotSpeeds = getChassisSpeeds();
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, getRotation());
    currentFieldVelocity =
        new Translation2d(
            fieldVelocityXFilter.calculate(fieldSpeeds.vxMetersPerSecond),
            fieldVelocityYFilter.calculate(fieldSpeeds.vyMetersPerSecond));
  }

  /**
   * Calculates translation scaling to ensure rotation requested is fully achieved.
   *
   * @param speeds The desired chassis speeds.
   * @return The scaled chassis speeds with rotation priority.
   */
  public ChassisSpeeds applyRotationPriority(ChassisSpeeds speeds) {
    double maxSpeed = MAX_LINEAR_VELOCITY.in(MetersPerSecond);

    double maxRotModuleSpeed = Math.abs(speeds.omegaRadiansPerSecond) * maxModuleRadius;

    double remainingBudget = Math.max(0.0, maxSpeed - maxRotModuleSpeed);
    double requestedTransSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double transScale =
        (requestedTransSpeed > 1e-6) ? Math.min(1.0, remainingBudget / requestedTransSpeed) : 1.0;

    return new ChassisSpeeds(
        speeds.vxMetersPerSecond * transScale,
        speeds.vyMetersPerSecond * transScale,
        speeds.omegaRadiansPerSecond);
  }

  /**
   * The single entry point for all robot movement.
   *
   * @param speeds The desired linear/angular velocity (robot-relative).
   * @param targetHeading If non-null, the robot will ignore speeds.omega and calculate its own
   *     rotation with PRIORITY BUDGETING.
   * @param omegaFF Angular velocity feedforward required to track a moving heading (rad/s).
   */
  public void runVelocity(ChassisSpeeds speeds, Rotation2d targetHeading, double omegaFF) {
    ChassisSpeeds finalSpeeds;

    if (targetHeading != null) {
      double rotationFeedback = calculateRotationFeedback(targetHeading) + omegaFF;

      // Recover field-relative translation to avoid coordinate corruption that causes veering
      ChassisSpeeds fieldSpeeds =
          ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getPose().getRotation());

      // We reconstruct the robot-relative speeds for the kinematics
      ChassisSpeeds robotSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              fieldSpeeds.vxMetersPerSecond,
              fieldSpeeds.vyMetersPerSecond,
              rotationFeedback,
              getPose().getRotation());

      finalSpeeds = applyRotationPriority(robotSpeeds);
    } else {
      lastTargetHeading = null;
      finalSpeeds = speeds;
    }

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(finalSpeeds, 0.02);
    runVelocityRaw(discreteSpeeds);
  }

  /**
   * Main entry point for movement. This acts as a wrapper that modifies the incoming speeds based
   * on the current Goal state.
   *
   * @param speeds Speeds relative to the robot's chassis.
   */
  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, headingOverrideSupplier.get(), headingVelocityOverrideSupplier.get());
  }

  /**
   * Internal method to run velocity without re-discretizing
   *
   * @param speeds Speeds in meters/sec, relative to the robot's chassis.
   */
  public void runVelocityRaw(ChassisSpeeds speeds) {
    int healthyMask = 0;

    for (int i = 0; i < 4; i++) {
      boolean driveOK = modules[i].isDriveMotorHealthy();
      boolean turnOK = modules[i].isTurnMotorHealthy();
      boolean encOK = modules[i].isTurnEncoderHealthy();

      isCaster[i] = false;
      isOpportunistic[i] = false;
      isDead[i] = false;

      if (!encOK) {
        isDead[i] = true;
      } else if (driveOK && turnOK) {
        healthyMask |= (1 << i);
      } else if (!driveOK && turnOK) {
        isCaster[i] = true;
      } else if (driveOK && !turnOK) {
        isOpportunistic[i] = true;
      } else {
        isDead[i] = true;
      }
    }

    SwerveModuleState[] idealStates = kinematics.toSwerveModuleStates(speeds);

    SwerveModuleState[] balancedStates = null;
    if (healthyMask > 0) {
      balancedStates = dynamicKinematics[healthyMask].toSwerveModuleStates(speeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(
          balancedStates, MAX_LINEAR_VELOCITY.in(MetersPerSecond));
    }

    int balancedIdx = 0;
    for (int i = 0; i < 4; i++) {
      if (isDead[i]) {
        // Avoid allocating a new SwerveModuleState; reuse the preallocated slot.
        finalStates[i].speedMetersPerSecond = 0.0;
        finalStates[i].angle = new Rotation2d(modules[i].getCurrentAngle());
        modules[i].stop();
      } else if (isCaster[i]) {
        finalStates[i].speedMetersPerSecond = 0.0;
        finalStates[i].angle = idealStates[i].angle;
        modules[i].runSetpoint(finalStates[i]);
      } else if (isOpportunistic[i]) {
        finalStates[i].speedMetersPerSecond = idealStates[i].speedMetersPerSecond;
        finalStates[i].angle = idealStates[i].angle;
        modules[i].runSetpoint(finalStates[i]);
      } else {
        finalStates[i].speedMetersPerSecond = balancedStates[balancedIdx].speedMetersPerSecond;
        finalStates[i].angle = balancedStates[balancedIdx].angle;
        modules[i].runSetpoint(finalStates[i]);
        balancedIdx++;
      }
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", finalStates);
    Logger.recordOutput("Drive/FaultTolerantMask", healthyMask);
  }

  /**
   * Runs the drive in a straight line with the specified drive output. Used for SysId.
   *
   * @param output The voltage to apply to the drive motors.
   */
  public void runCharacterization(Voltage output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = MODULE_TRANSLATIONS[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Returns a command to run a quasistatic test in the specified direction for characterization.
   *
   * @param direction The direction to run the test.
   * @return A Command to execute the test.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(Volts.zero()))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /**
   * Returns a command to run a dynamic test in the specified direction for characterization.
   *
   * @param direction The direction to run the test.
   * @return A Command to execute the test.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(Volts.zero()))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the modules.
   *
   * @return The current state of each module.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    for (int i = 0; i < 4; i++) {
      currentStates[i] = modules[i].getState();
    }
    return currentStates;
  }

  /**
   * Returns the module positions (turn angles and drive positions) for all of the modules.
   *
   * @return The current position of each module.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /**
   * Returns the measured chassis speeds of the robot based on module states.
   *
   * @return The current robot chassis speeds.
   */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the field-relative velocity of the robot as a Translation2d (x = m/s forward on field,
   * y = m/s left on field).
   *
   * @return The current field-relative linear velocity.
   */
  public Translation2d getFieldRelativeVelocity() {
    return currentFieldVelocity;
  }

  /**
   * Returns the position of each module in radians for characterization.
   *
   * @return An array of module positions in radians.
   */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition().in(Radians);
    }
    return values;
  }

  /**
   * Returns the average velocity of the modules for characterization.
   *
   * @return The average module velocity in radians per second.
   */
  public AngularVelocity getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity().in(RadiansPerSecond) / 4.0;
    }
    return RadiansPerSecond.of(output);
  }

  /**
   * Returns the current odometry pose.
   *
   * @return The estimated pose of the robot on the field.
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the current yaw (heading) velocity of the robot from the gyro.
   *
   * @return The current angular velocity around the yaw axis in radians per second.
   */
  public double getYawVelocityRadPerSec() {
    return gyroInputs.yawVelocity.in(RadiansPerSecond);
  }

  /**
   * Returns the current rotation of the robot compensated for latency. This is done by taking the
   * current gyro rotation and adding the product of the yaw velocity and an estimated latency to
   * predict where the robot will be when the module measurements are processed.
   *
   * @return The latency-compensated rotation of the robot.
   */
  public Rotation2d getLatCompRotation() {
    double latency = 0.020;
    return getRotation().plus(Rotation2d.fromRadians(getYawVelocityRadPerSec() * latency));
  }

  /**
   * Returns the current odometry rotation.
   *
   * @return The estimated rotation of the robot.
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Resets the current odometry pose to a specific position. This assumes the bot is flat on the
   * ground and will break horrendously if not.
   *
   * @param pose The new pose to set the robot to.
   */
  public void setPose(Pose2d pose) {
    // We have to add 5ms delay to ensure the gyro has time to update with the new yaw before we
    // read it for odometry.
    lastResetTimestamp = (RobotController.getFPGATime() / 1.0e6) + 0.005;

    gyroIO.setRotation(new Rotation3d(pose.getRotation()));
    rawGyroRotation = pose.getRotation();

    gyroHistory.clear();
    gyroHistory.addSample(lastResetTimestamp, rawGyroRotation);

    SwerveModulePosition[] currentPositions = getModulePositions();
    for (int i = 0; i < 4; i++) {
      lastModulePositions[i].distanceMeters = currentPositions[i].distanceMeters;
      lastModulePositions[i].angle = currentPositions[i].angle;
    }

    poseEstimator.resetPosition(rawGyroRotation, currentPositions, pose);
  }

  /**
   * Adds a new timestamped vision measurement to the pose estimator.
   *
   * @param visionRobotPoseMeters The pose reported by the vision system.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs The standard deviations of the vision measurement.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /**
   * Returns the maximum linear velocity.
   *
   * @return The maximum linear velocity.
   */
  public LinearVelocity getMaxLinearVelocity() {
    return MAX_LINEAR_VELOCITY;
  }

  /**
   * Returns the maximum angular velocity.
   *
   * @return The maximum angular velocity.
   */
  public AngularVelocity getMaxAngularVelocity() {
    return RadiansPerSecond.of(
        MAX_LINEAR_VELOCITY.in(MetersPerSecond) / DRIVE_BASE_RADIUS.in(Meters));
  }

  /**
   * Returns the swerve drive kinematics instance for this drivetrain.
   *
   * @return The SwerveDriveKinematics used by the drive subsystem.
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Checks if the robot is aligned within a certain tolerance to a target angle.
   *
   * @param targetAngle The angle to check against.
   * @return True if within tolerance, false otherwise.
   */
  public boolean isAlignedWithTarget(Rotation2d targetAngle) {
    return Math.abs(getRotation().minus(targetAngle).getRadians())
        < DriveConstants.HEADING_ALIGNMENT_TOLERANCE.in(Radians);
  }

  /** Resets the rotation PID to prevent sudden jerks when taking over heading control. */
  public void resetRotationController() {
    rotationController.reset();
  }

  /**
   * Calculates the closed-loop angular velocity required to reach the target heading.
   *
   * @param targetHeading The target heading as a Rotation2d.
   * @return The required angular velocity.
   */
  public double calculateRotationFeedback(Rotation2d targetHeading) {
    double rawTargetVelocity = 0.0;
    if (lastTargetHeading != null) {
      rawTargetVelocity = targetHeading.minus(lastTargetHeading).getRadians() / 0.02;
    }
    lastTargetHeading = targetHeading;

    // Smooth out the noisy derivative
    double smoothedTargetVelocity = targetVelocityFilter.calculate(rawTargetVelocity);

    double correction =
        rotationController.calculate(getLatCompRotation().getRadians(), targetHeading.getRadians());

    return MathUtil.clamp(correction, -7.0, 7.0) + smoothedTargetVelocity;
  }

  /**
   * Returns whether or not the subsystem is healthy
   *
   * @return True if the subsystem is healthy, false otherwise.
   */
  public boolean isHealthy() {
    if (!gyroInputs.healthy) {
      return false;
    }
    for (var module : modules) {
      if (!module.isHealthy()) {
        return false;
      }
    }
    return true;
  }

  /** Clears all faults and warnings. */
  public void clearFaults() {
    gyroIO.clearFaults();
    for (var module : modules) {
      module.clearFaults();
    }
  }
}
