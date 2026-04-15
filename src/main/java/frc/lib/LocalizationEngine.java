// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

/**
 * The Localization Engine is a high-performance, low-allocation state estimator. It fuses swerve
 * odometry, dual-gyro heading, and asynchronous vision data.
 */
public class LocalizationEngine {
  private static final double HISTORY_DURATION_SEC = 2.0;

  // Robot State Buffers
  private final double[] histX;
  private final double[] histY;
  private final double[] histTheta;
  private final double[] histVx;
  private final double[] histVy;
  private final double[] histTime;
  private final int historySize;

  private int head = 0;
  private int count = 0;

  // Current State Scalars
  private double curX, curY, curTheta;
  private double curVx, curVy, curOmega;
  private double curAx, curAy;

  // Filters to smooth the acceleration derivative.
  private final LinearFilter xAccelFilter;
  private final LinearFilter yAccelFilter;

  // Cached objects
  private Pose2d latestPose = Pose2d.kZero;
  private ChassisSpeeds latestVelocity = new ChassisSpeeds();
  private ChassisSpeeds latestAcceleration = new ChassisSpeeds();

  public LocalizationEngine(Pose2d initialPose, Frequency odometryFrequency) {
    this.historySize = (int) (odometryFrequency.in(Hertz) * HISTORY_DURATION_SEC);

    this.histX = new double[historySize];
    this.histY = new double[historySize];
    this.histTheta = new double[historySize];
    this.histVx = new double[historySize];
    this.histVy = new double[historySize];
    this.histTime = new double[historySize];

    double dt = 1.0 / odometryFrequency.in(Hertz);
    this.xAccelFilter = LinearFilter.singlePoleIIR(0.1, dt);
    this.yAccelFilter = LinearFilter.singlePoleIIR(0.1, dt);

    reset(initialPose);
  }

  /**
   * Resets the Engine to a known Pose.
   *
   * @param pose The initial pose to reset to.
   */
  public void reset(Pose2d pose) {
    curX = pose.getX();
    curY = pose.getY();
    curTheta = pose.getRotation().getRadians();

    curVx = curVy = curOmega = 0;
    curAx = curAy = 0;
    head = 1;
    count = 1;

    // Reset filters
    xAccelFilter.reset();
    yAccelFilter.reset();

    // Seed the first history frame properly to prevent timestamp jump bugs
    histX[0] = curX;
    histY[0] = curY;
    histTheta[0] = curTheta;
    histVx[0] = 0;
    histVy[0] = 0;
    histTime[0] = Timer.getFPGATimestamp();

    latestPose = pose;
    latestVelocity = new ChassisSpeeds();
    latestAcceleration = new ChassisSpeeds();
  }

  /**
   * Updates the Engine using Second-Order Kinematic Integration.
   *
   * @param timestamp FPGA Timestamp in seconds.
   * @param gyro The current "Ground Truth" gyro rotation.
   * @param twist The Twist2d since the last tick (Kept for API compatibility, unused internally).
   * @param robotSpeeds The current instantaneous robot speeds from forward kinematics.
   * @param measuredOmega The measured angular velocity from the gyro.
   */
  public void update(
      double timestamp,
      Rotation2d gyro,
      Twist2d twist,
      ChassisSpeeds robotSpeeds,
      double measuredOmega) {
    double prevVx = curVx;
    double prevVy = curVy;
    double prevTheta = curTheta;

    int prevIdx = (head - 1 + this.historySize) % this.historySize;
    double dt = (count > 1) ? (timestamp - histTime[prevIdx]) : 0.005;

    // Calculate Heading
    curTheta = gyro.getRadians();
    double dtheta = MathUtil.angleModulus(curTheta - prevTheta);

    // Rotate instant Robot-Relative speeds to Field-Relative using Average Heading
    double avgTheta = prevTheta + (dtheta * 0.5);
    double cosA = Math.cos(avgTheta);
    double sinA = Math.sin(avgTheta);

    curVx = robotSpeeds.vxMetersPerSecond * cosA - robotSpeeds.vyMetersPerSecond * sinA;
    curVy = robotSpeeds.vxMetersPerSecond * sinA + robotSpeeds.vyMetersPerSecond * cosA;
    curOmega = measuredOmega;

    // Second-Order Pose Integration & Acceleration Filtering
    if (dt > 1E-5) {
      // Integrate velocity over time to find position displacement
      curX += (curVx + prevVx) * 0.5 * dt;
      curY += (curVy + prevVy) * 0.5 * dt;

      // Derive and filter acceleration
      double rawAx = (curVx - prevVx) / dt;
      double rawAy = (curVy - prevVy) / dt;

      curAx = xAccelFilter.calculate(rawAx);
      curAy = yAccelFilter.calculate(rawAy);
    }

    // Store in Circular Buffer
    histX[head] = curX;
    histY[head] = curY;
    histTheta[head] = curTheta;
    histVx[head] = curVx;
    histVy[head] = curVy;
    histTime[head] = timestamp;

    head = (head + 1) % this.historySize;
    if (count < this.historySize) count++;

    // Update cached objects for main loop access
    latestPose = new Pose2d(curX, curY, gyro);
    latestVelocity = new ChassisSpeeds(curVx, curVy, curOmega);
    latestAcceleration = new ChassisSpeeds(curAx, curAy, 0);
  }

  /**
   * Vision Fusion. Corrects the global position while accounting for rotation during the latency
   * period.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    if (count == 0) return;

    int olderIdx = -1;
    int newerIdx = -1;

    // Search backwards through history to find the bounding frames
    for (int i = 1; i <= count; i++) {
      int idx = (head - i + this.historySize) % this.historySize;

      if (histTime[idx] <= timestamp) {
        olderIdx = idx;
        newerIdx = (i == 1) ? idx : (head - (i - 1) + this.historySize) % this.historySize;
        break;
      }
    }

    // If the measurement is older than our entire 2-second history buffer, discard it
    if (olderIdx == -1) return;

    double interpX, interpY, interpTheta;

    // Interpolate the state
    if (olderIdx == newerIdx) {
      interpX = histX[olderIdx];
      interpY = histY[olderIdx];
      interpTheta = histTheta[olderIdx];
    } else {
      double t0 = histTime[olderIdx];
      double t1 = histTime[newerIdx];
      double alpha = (timestamp - t0) / (t1 - t0); // Percentage between the two frames

      // Linear interpolation for X and Y
      interpX = MathUtil.interpolate(histX[olderIdx], histX[newerIdx], alpha);
      interpY = MathUtil.interpolate(histY[olderIdx], histY[newerIdx], alpha);

      // Spherical linear interpolation (Slerp) for Theta to prevent -Pi / Pi wrap-around bugs
      Rotation2d rot0 = Rotation2d.fromRadians(histTheta[olderIdx]);
      Rotation2d rot1 = Rotation2d.fromRadians(histTheta[newerIdx]);
      interpTheta = rot0.interpolate(rot1, alpha).getRadians();
    }

    // Calculate Raw Error in Field Frame using the Interpolated Pose
    double errX = visionPose.getX() - interpX;
    double errY = visionPose.getY() - interpY;

    // Transform Field Error to Robot-Local Error at the time of the vision frame
    double cosOld = Math.cos(-interpTheta);
    double sinOld = Math.sin(-interpTheta);
    double localErrX = errX * cosOld - errY * sinOld;
    double localErrY = errX * sinOld + errY * cosOld;

    // Apply Kalman trust multipliers
    localErrX *= 1.0 / (1.0 + stdDevs.get(0, 0));
    localErrY *= 1.0 / (1.0 + stdDevs.get(1, 0));

    // Transform Local Error BACK to Field Error using the CURRENT heading
    double cosCur = Math.cos(curTheta);
    double sinCur = Math.sin(curTheta);
    double fieldErrX = localErrX * cosCur - localErrY * sinCur;
    double fieldErrY = localErrX * sinCur + localErrY * cosCur;

    // Shift the current state and the entire history buffer by the rotated error
    curX += fieldErrX;
    curY += fieldErrY;

    for (int i = 0; i < count; i++) {
      int idx = (head - 1 - i + this.historySize) % this.historySize;
      histX[idx] += fieldErrX;
      histY[idx] += fieldErrY;
    }

    latestPose = new Pose2d(curX, curY, Rotation2d.fromRadians(curTheta));
  }

  /** Returns the current estimated Pose. */
  public Pose2d getPose() {
    return latestPose;
  }

  /** Returns the Field-Relative velocity (m/s). */
  public ChassisSpeeds getFieldVelocity() {
    return latestVelocity;
  }

  /** Returns the Field-Relative acceleration (m/s^2). */
  public ChassisSpeeds getFieldAcceleration() {
    return latestAcceleration;
  }

  /** Returns the ROBOT-RELATIVE velocity (for PathPlanner and Drivers). */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(latestVelocity, latestPose.getRotation());
  }

  /**
   * Predicts where the robot will be in the specified number of seconds. Uses current position,
   * velocity, and acceleration for a second-order kinematic prediction.
   *
   * @param lookahead How far into the future to predict.
   * @return The predicted Pose2d of the robot.
   */
  public Pose2d predictPose(Time lookahead) {
    Pose2d pose = latestPose;
    ChassisSpeeds vel = latestVelocity;
    ChassisSpeeds accel = latestAcceleration;

    double lookaheadSeconds = lookahead.in(Seconds);
    double t2 = lookaheadSeconds * lookaheadSeconds;

    // Kinematic calculations
    double predX =
        pose.getX()
            + (vel.vxMetersPerSecond * lookaheadSeconds)
            + (0.5 * accel.vxMetersPerSecond * t2);
    double predY =
        pose.getY()
            + (vel.vyMetersPerSecond * lookaheadSeconds)
            + (0.5 * accel.vyMetersPerSecond * t2);

    // Constant angular velocity prediction
    double predTheta =
        pose.getRotation().getRadians() + (vel.omegaRadiansPerSecond * lookaheadSeconds);

    return new Pose2d(predX, predY, Rotation2d.fromRadians(predTheta));
  }
}
