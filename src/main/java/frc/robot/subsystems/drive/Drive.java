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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.FaultUtil;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for the robot's swerve drive.
 *
 * <p>Handles kinematics, odometry estimation (incorporating gyro, modules, and vision), and
 * interfacing with PathPlanner for autonomous paths.
 */
public class Drive extends SubsystemBase {
  /** Lock used to synchronize access to odometry data between the main loop and sampling thread. */
  public static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private final Alert gyroFaultAlert = new Alert("Gyro fault detected.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;

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

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().startThread();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
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
  }

  @Override
  public void periodic() {
    odometryLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);

      for (int i = 0; i < gyroInputs.odometryYawTimestamps.length; i++) {
        gyroHistory.addSample(
            gyroInputs.odometryYawTimestamps[i],
            Rotation2d.fromRadians(gyroInputs.odometryYawPositions[i]));
      }

      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (var module : modules) {
        module.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
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
      var interpolatedRotation = gyroHistory.getSample(timestamp);

      if (interpolatedRotation.isPresent()) {
        rawGyroRotation = interpolatedRotation.get();
      } else {
        Twist2d twist = kinematics.toTwist2d(odometryDeltasBuffer);
        rawGyroRotation = rawGyroRotation.plus(Rotation2d.fromRadians(twist.dtheta));
      }

      poseEstimator.updateWithTime(timestamp, rawGyroRotation, odometryPositionsBuffer);
    }

    // Update gyro alert
    boolean gyroConnected = gyroInputs.connected && Constants.CURRENT_MODE != Mode.SIM;
    gyroDisconnectedAlert.set(!gyroConnected);

    // Update gyro fault alert
    if (gyroConnected) {
      gyroFaultAlert.set(!gyroInputs.healthy);
      if (!gyroInputs.healthy) {
        gyroFaultAlert.setText(
            FaultUtil.getArrayString("Gyro Faults: ", gyroInputs.unhealthyReasons));
      }
    } else {
      gyroFaultAlert.set(false);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec, relative to the robot's field-centric perspective.
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, MAX_LINEAR_VELOCITY.in(MetersPerSecond));

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
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
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
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
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the field-relative velocity of the robot as a Translation2d (x = m/s forward on field,
   * y = m/s left on field).
   *
   * @return The current field-relative linear velocity.
   */
  public Translation2d getFieldRelativeVelocity() {
    ChassisSpeeds robotSpeeds = getChassisSpeeds();
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, getRotation());
    return new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
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
   * Returns the current odometry rotation.
   *
   * @return The estimated rotation of the robot.
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Resets the current odometry pose to a specific position.
   *
   * @param pose The new pose to set the robot to.
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
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
   * @param tolerance The allowed angular error.
   * @return True if within tolerance, false otherwise.
   */
  public boolean isAlignedWithTarget(Rotation2d targetAngle, Rotation2d tolerance) {
    return Math.abs(getRotation().minus(targetAngle).getRadians()) < tolerance.getRadians();
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
    for (var module : modules) {
      module.clearFaults();
    }
  }
}
