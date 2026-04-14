// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A hyper-optimized, zero-allocation Pose Estimator designed specifically for high-frequency
 * odometry (200Hz) and asynchronous vision updates.
 *
 * <p>Unlike WPILib's SwerveDrivePoseEstimator, this class does not use TreeMaps or allocate objects
 * during standard execution. It maintains history using primitive arrays and applies vision
 * corrections by shifting the entire timeline, projecting error into the robot's local frame to
 * perfectly account for curvature and rotation during vision latency.
 */
public class PoseEstimator {
  private static final int HISTORY_SIZE = 400; // 2 seconds of history at 200Hz

  // Primitive Circular Buffer for Zero-GC History
  private final double[] histX = new double[HISTORY_SIZE];
  private final double[] histY = new double[HISTORY_SIZE];
  private final double[] histTheta = new double[HISTORY_SIZE];
  private final double[] histTime = new double[HISTORY_SIZE];

  private int head = 0;
  private int count = 0;

  // Ground Truth State
  private double curX;
  private double curY;
  private double curTheta;

  // Pre-allocated return object
  private Pose2d latestPose = Pose2d.kZero;

  /**
   * Constructs the estimator with an initial pose.
   *
   * @param initialPose The starting pose of the robot.
   */
  public PoseEstimator(Pose2d initialPose) {
    resetPosition(initialPose.getRotation(), initialPose);
  }

  /**
   * Resets the estimator to a specific position.
   *
   * @param gyroAngle The current gyro angle.
   * @param newPose The new pose to snap to.
   */
  public void resetPosition(Rotation2d gyroAngle, Pose2d newPose) {
    curX = newPose.getX();
    curY = newPose.getY();
    curTheta = gyroAngle.getRadians();

    head = 0;
    count = 1;

    histX[0] = curX;
    histY[0] = curY;
    histTheta[0] = curTheta;
    histTime[0] = 0.0;

    latestPose = newPose;
  }

  /**
   * Updates the estimator with a new 200Hz odometry tick.
   *
   * @param timestamp The exact timestamp of this odometry tick.
   * @param gyroAngle The current "Ground Truth" gyro rotation.
   * @param twist The Twist2d representing the delta since the last tick (from kinematics).
   */
  public void update(double timestamp, Rotation2d gyroAngle, Twist2d twist) {
    // Use gyro for precise rotation tracking
    double dtheta = MathUtil.angleModulus(gyroAngle.getRadians() - curTheta);

    // Exact Pose Exponential Integration (Constant Curvature)
    double sin = Math.sin(dtheta);
    double cos = Math.cos(dtheta);
    double s, c;

    // Taylor series approximation for extremely small turns
    if (Math.abs(dtheta) < 1E-9) {
      s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
      c = 0.5 * dtheta;
    } else {
      s = sin / dtheta;
      c = (1.0 - cos) / dtheta;
    }

    // Transform local twist to global movement
    double xOffset = twist.dx * s - twist.dy * c;
    double yOffset = twist.dx * c + twist.dy * s;

    double cosTheta = Math.cos(curTheta);
    double sinTheta = Math.sin(curTheta);

    // Step current pose forward
    curX += xOffset * cosTheta - yOffset * sinTheta;
    curY += xOffset * sinTheta + yOffset * cosTheta;
    curTheta = gyroAngle.getRadians();

    // Store in circular buffer
    histX[head] = curX;
    histY[head] = curY;
    histTheta[head] = curTheta;
    histTime[head] = timestamp;

    head = (head + 1) % HISTORY_SIZE;
    if (count < HISTORY_SIZE) {
      count++;
    }

    // Update the cached return object safely
    latestPose = new Pose2d(curX, curY, gyroAngle);
  }

  /**
   * Applies an asynchronous vision measurement to the pose history.
   *
   * @param visionPose The pose returned by the vision system.
   * @param timestamp The timestamp of the vision frame.
   * @param stdDevs The standard deviations (trust) of the vision measurement.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    if (count == 0) return; // Ignore if odometry hasn't ticked yet

    // Fast Linear Backward Scan to find history bounding the timestamp
    int bestIdx = -1;
    for (int i = 1; i <= count; i++) {
      int idx = (head - i + HISTORY_SIZE) % HISTORY_SIZE;
      if (histTime[idx] <= timestamp) {
        bestIdx = idx;
        break;
      }
    }

    // Interpolate History at the vision timestamp
    double interpX, interpY, interpTheta;
    if (bestIdx == -1) {
      // Vision is older than history buffer; snap to oldest
      int oldestIdx = (head - count + HISTORY_SIZE) % HISTORY_SIZE;
      interpX = histX[oldestIdx];
      interpY = histY[oldestIdx];
      interpTheta = histTheta[oldestIdx];
    } else if (bestIdx == (head - 1 + HISTORY_SIZE) % HISTORY_SIZE) {
      // Vision is newer than or equal to current; snap to current
      interpX = histX[bestIdx];
      interpY = histY[bestIdx];
      interpTheta = histTheta[bestIdx];
    } else {
      // Interpolate between bestIdx and the chronologically next frame
      int nextIdx = (bestIdx + 1) % HISTORY_SIZE;
      double t0 = histTime[bestIdx];
      double dt = histTime[nextIdx] - t0;

      if (dt <= 1e-6) {
        interpX = histX[bestIdx];
        interpY = histY[bestIdx];
        interpTheta = histTheta[bestIdx];
      } else {
        double alpha = MathUtil.clamp((timestamp - t0) / dt, 0.0, 1.0);
        interpX = histX[bestIdx] + alpha * (histX[nextIdx] - histX[bestIdx]);
        interpY = histY[bestIdx] + alpha * (histY[nextIdx] - histY[bestIdx]);

        double dTheta = MathUtil.angleModulus(histTheta[nextIdx] - histTheta[bestIdx]);
        interpTheta = histTheta[bestIdx] + alpha * dTheta;
      }
    }

    // Find raw error in Field Coordinate Frame
    double errX = visionPose.getX() - interpX;
    double errY = visionPose.getY() - interpY;

    // Transform error into Robot-Local Coordinate Frame (crucial for latency accuracy)
    double cosT = Math.cos(-interpTheta);
    double sinT = Math.sin(-interpTheta);
    double localErrX = errX * cosT - errY * sinT;
    double localErrY = errX * sinT + errY * cosT;

    // Calculate Kalman-style Gain based on Standard Deviations (Trust)
    double kX = 1.0 / (1.0 + stdDevs.get(0, 0));
    double kY = 1.0 / (1.0 + stdDevs.get(1, 0));

    localErrX *= kX;
    localErrY *= kY;

    // Project Local Error BACK out to Field Frame using the CURRENT heading
    double cosCur = Math.cos(curTheta);
    double sinCur = Math.sin(curTheta);
    double fieldErrX = localErrX * cosCur - localErrY * sinCur;
    double fieldErrY = localErrX * sinCur + localErrY * cosCur;

    // Apply Shift to Current Pose
    curX += fieldErrX;
    curY += fieldErrY;

    // Shift ENTIRE History Buffer (Zero-Allocation Timeline Rewrite)
    for (int i = 0; i < count; i++) {
      histX[i] += fieldErrX;
      histY[i] += fieldErrY;
    }
  }

  /**
   * Returns the current estimated position.
   *
   * @return The latest Pose2d.
   */
  public Pose2d getEstimatedPosition() {
    return latestPose;
  }
}
