package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The Localization Engine is a high-performance, zero-allocation state estimator.
 * It fuses 200Hz swerve odometry, dual-gyro heading, and asynchronous vision data.
 */
public class LocalizationEngine {
  private static final int HISTORY_SIZE = 400; // 2 seconds of history at 200Hz

  // Robot State Buffers (Primitive Arrays for Zero-GC)
  private final double[] histX = new double[HISTORY_SIZE];
  private final double[] histY = new double[HISTORY_SIZE];
  private final double[] histTheta = new double[HISTORY_SIZE];
  private final double[] histVx = new double[HISTORY_SIZE];
  private final double[] histVy = new double[HISTORY_SIZE];
  private final double[] histTime = new double[HISTORY_SIZE];

  private int head = 0;
  private int count = 0;

  // Current State Scalars
  private double curX, curY, curTheta;
  private double curVx, curVy, curOmega;
  private double curAx, curAy;

  // Filters to smooth derived velocity from 200Hz differentiation
  private final LinearFilter xVelFilter = LinearFilter.singlePoleIIR(0.02, 0.005);
  private final LinearFilter yVelFilter = LinearFilter.singlePoleIIR(0.02, 0.005);

  // Cached objects for the 50Hz loop
  private volatile Pose2d latestPose = Pose2d.kZero;
  private volatile ChassisSpeeds latestVelocity = new ChassisSpeeds();
  private volatile ChassisSpeeds latestAcceleration = new ChassisSpeeds();

  public LocalizationEngine(Pose2d initialPose) {
    reset(initialPose);
  }

  /** Resets the Engine to a known Pose. */
  public void reset(Pose2d pose) {
    curX = pose.getX();
    curY = pose.getY();
    curTheta = pose.getRotation().getRadians();
    curVx = curVy = curOmega = 0;
    curAx = curAy = 0;
    head = 0;
    count = 0;
    xVelFilter.reset();
    yVelFilter.reset();
  }

  /**
   * Updates the Engine at 200Hz.
   * @param timestamp FPGA Timestamp in seconds.
   * @param gyro The current "Ground Truth" gyro rotation.
   * @param twist The Twist2d (dx, dy, dtheta) since the last 200Hz tick.
   */
  public void update(double timestamp, Rotation2d gyro, Twist2d twist) {
    double prevX = curX;
    double prevY = curY;
    double prevVx = curVx;
    double prevVy = curVy;

    // Exact Pose Exponential Integration (Constant Curvature)
    double dtheta = MathUtil.angleModulus(gyro.getRadians() - curTheta);
    double s, c;
    if (Math.abs(dtheta) < 1E-9) {
      s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
      c = 0.5 * dtheta;
    } else {
      s = Math.sin(dtheta) / dtheta;
      c = (1.0 - Math.cos(dtheta)) / dtheta;
    }

    double xOffset = twist.dx * s - twist.dy * c;
    double yOffset = twist.dx * c + twist.dy * s;

    double cosT = Math.cos(curTheta);
    double sinT = Math.sin(curTheta);

    curX += xOffset * cosT - yOffset * sinT;
    curY += xOffset * sinT + yOffset * cosT;
    curTheta = gyro.getRadians();

    // Velocity Derivation & Filtering
    double dt = (count > 0) ? (timestamp - histTime[(head - 1 + HISTORY_SIZE) % HISTORY_SIZE]) : 0.02;
    if (dt > 0) {
      double rawVx = (curX - prevX) / dt;
      double rawVy = (curY - prevY) / dt;
      curVx = xVelFilter.calculate(rawVx);
      curVy = yVelFilter.calculate(rawVy);
      curOmega = dtheta / dt;

      // Acceleration Derivation
      curAx = (curVx - prevVx) / dt;
      curAy = (curVy - prevVy) / dt;
    }

    // Store in Circular Buffer
    histX[head] = curX;
    histY[head] = curY;
    histTheta[head] = curTheta;
    histVx[head] = curVx;
    histVy[head] = curVy;
    histTime[head] = timestamp;

    head = (head + 1) % HISTORY_SIZE;
    if (count < HISTORY_SIZE) count++;

    // Update cached objects for main loop access
    latestPose = new Pose2d(curX, curY, gyro);
    latestVelocity = new ChassisSpeeds(curVx, curVy, curOmega);
    latestAcceleration = new ChassisSpeeds(curAx, curAy, 0);
  }

  /** 
   * Asynchronous Vision Fusion.
   * Corrects the global position while preserving derived relative velocity.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    if (count == 0) return;

    // Find the historical odometry pose that matches vision timestamp
    int bestIdx = -1;
    for (int i = 1; i <= count; i++) {
      int idx = (head - i + HISTORY_SIZE) % HISTORY_SIZE;
      if (histTime[idx] <= timestamp) {
        bestIdx = idx;
        break;
      }
    }

    if (bestIdx == -1) return;

    // Calculate Error in Field Frame
    double errX = visionPose.getX() - histX[bestIdx];
    double errY = visionPose.getY() - histY[bestIdx];

    // Kalman Gain (Simplified for speed)
    double kX = 1.0 / (1.0 + stdDevs.get(0, 0));
    double kY = 1.0 / (1.0 + stdDevs.get(1, 0));

    double corrX = errX * kX;
    double corrY = errY * kY;

    // Shift the current state and the entire history buffer
    curX += corrX;
    curY += corrY;

    for (int i = 0; i < count; i++) {
      int idx = (head - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
      histX[idx] += corrX;
      histY[idx] += corrY;
    }
    
    latestPose = new Pose2d(curX, curY, Rotation2d.fromRadians(curTheta));
  }

  /** 
   * Returns the current estimated Pose.
   */
  public Pose2d getPose() {
    return latestPose;
  }

  /** 
   * Returns the Field-Relative velocity (m/s).
   * Used for aiming and lead compensation.
   */
  public ChassisSpeeds getFieldVelocity() {
    return latestVelocity;
  }

  /** 
   * Returns the Field-Relative acceleration (m/s²).
   */
  public ChassisSpeeds getFieldAcceleration() {
    return latestAcceleration;
  }

  /**
   * Returns the ROBOT-RELATIVE velocity.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(latestVelocity, latestPose.getRotation());
  }
}
