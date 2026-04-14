// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/** A specialized simulation for a spring-extended, motor-retracted climber. */
public class ClimbSim extends LinearSystemSim<N2, N1, N2> {
  private final DCMotor m_gearbox;

  // Physics plants for the two different mass states
  private final LinearSystem<N2, N1, N2> m_plantExtending;
  private final LinearSystem<N2, N1, N2> m_plantClimbing;

  private final double m_minHeight;
  private final double m_maxHeight;
  private final double m_springForceNewtons;
  private final double m_armMassKg;
  private final double m_botMassKg;

  private boolean m_isHooked = false;

  /**
   * Constructs a new ClimbSim.
   *
   * @param gearbox The climber motor gearbox.
   * @param gearing The total reduction (e.g., 50.0).
   * @param armMassKg The mass of the extending climber stage.
   * @param botMassKg The total mass of the robot.
   * @param drumRadiusMeters The radius of the spool/drum.
   * @param minHeightMeters The bottom limit (usually 0).
   * @param maxHeightMeters The height at which the hook catches the bar.
   * @param springForceNewtons The upward force provided by extension springs.
   */
  public ClimbSim(
      DCMotor gearbox,
      double gearing,
      double armMassKg,
      double botMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      double springForceNewtons) {
    super(LinearSystemId.createElevatorSystem(gearbox, armMassKg, drumRadiusMeters, gearing));

    this.m_gearbox = gearbox;
    this.m_armMassKg = armMassKg;
    this.m_botMassKg = botMassKg;
    this.m_minHeight = minHeightMeters;
    this.m_maxHeight = maxHeightMeters;
    this.m_springForceNewtons = springForceNewtons;

    this.m_plantExtending =
        LinearSystemId.createElevatorSystem(gearbox, armMassKg, drumRadiusMeters, gearing);
    this.m_plantClimbing =
        LinearSystemId.createElevatorSystem(gearbox, botMassKg, drumRadiusMeters, gearing);

    setState(0.0, 0.0);
  }

  /**
   * Sets the state of the simulation.
   *
   * @param positionMeters The new position in meters.
   * @param velocityMetersPerSecond The new velocity in meters per second.
   */
  public final void setState(double positionMeters, double velocityMetersPerSecond) {
    super.setState(VecBuilder.fill(positionMeters, velocityMetersPerSecond));
  }

  /**
   * Returns whether the climber has hooked onto the bar.
   *
   * @return True if hooked, false otherwise.
   */
  public boolean isHooked() {
    return m_isHooked;
  }

  /**
   * Returns the current position of the climber.
   *
   * @return The position in meters.
   */
  public double getPositionMeters() {
    return getOutput(0);
  }

  /**
   * Returns the current velocity of the climber.
   *
   * @return The velocity in meters per second.
   */
  public double getVelocityMetersPerSecond() {
    return getOutput(1);
  }

  /**
   * Sets the input voltage for the climber motors.
   *
   * @param volts The input voltage to apply.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
    clampInput(RobotController.getBatteryVoltage());
  }

  /**
   * Returns the simulated current draw of the climber.
   *
   * @return The current draw in amps.
   */
  public double getCurrentDrawAmps() {
    var plant = m_isHooked ? m_plantClimbing : m_plantExtending;
    double kA = 1 / plant.getB().get(1, 0);
    double kV = -plant.getA().get(1, 1) * kA;
    double motorVelocityRadPerSec =
        getVelocityMetersPerSecond() * kV * m_gearbox.KvRadPerSecPerVolt;
    var appliedVoltage = m_u.get(0, 0);
    return m_gearbox.getCurrent(motorVelocityRadPerSec, appliedVoltage)
        * Math.signum(appliedVoltage);
  }

  @Override
  protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
    // Permanent Hook Logic: If we ever reach the top, we are hooked forever.
    if (currentXhat.get(0, 0) >= m_maxHeight) {
      m_isHooked = true;
    }

    // Integration with custom forces
    var updatedXhat =
        NumericalIntegration.rkdp(
            (x, _u) -> {
              // Select the plant and mass based on hook state
              var plant = m_isHooked ? m_plantClimbing : m_plantExtending;
              double currentMass = m_isHooked ? m_botMassKg : m_armMassKg;

              // Basic motor physics: x_dot = Ax + Bu
              Matrix<N2, N1> xdot = plant.getA().times(x).plus(plant.getB().times(_u));

              // Constant downward acceleration
              xdot = xdot.plus(VecBuilder.fill(0, -9.8));

              // Constant upward acceleration (Force / Mass)
              double springAcc = m_springForceNewtons / currentMass;
              xdot = xdot.plus(VecBuilder.fill(0, springAcc));

              // If the motor is not being driven and the climber is nearly still,
              // the worm gear friction cancels out gravity and springs.
              if (Math.abs(_u.get(0, 0)) < 0.1 && Math.abs(x.get(1, 0)) < 0.05) {
                return VecBuilder.fill(x.get(1, 0), 0.0);
              }

              return xdot;
            },
            currentXhat,
            u,
            dtSeconds);

    // Clamp to physical limits
    double pos = updatedXhat.get(0, 0);
    if (pos <= m_minHeight) {
      return VecBuilder.fill(m_minHeight, 0.0);
    }
    // Only clamp the top if we aren't hooked (allows small movements at the bar)
    if (!m_isHooked && pos >= m_maxHeight) {
      return VecBuilder.fill(m_maxHeight, 0.0);
    }

    return updatedXhat;
  }
}
