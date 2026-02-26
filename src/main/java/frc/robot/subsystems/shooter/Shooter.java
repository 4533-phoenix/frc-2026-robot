package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final FlywheelIO flywheelIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private AngularVelocity targetVelocity = RadiansPerSecond.of(0.0);
  private boolean isShooting = false;

  private final Alert flywheelDisconnectedAlert =
      new Alert("Flywheel IO disconnected", AlertType.kWarning);

  public Shooter(FlywheelIO flywheelIO, HoodIO hoodIO) {
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    flywheelDisconnectedAlert.set(!flywheelInputs.connected);

    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
  }

  /** Commands the shooter mechanisms to match a calculated state. */
  public void setTargetState(ShooterState state) {
    targetVelocity = state.flywheelSpeed();
    flywheelIO.setAngularVelocity(state.flywheelSpeed());
    hoodIO.setLength(convertHoodAngleToServoLength(state.hoodAngle()));
    isShooting = true;
  }

  /** Converts the desired Hood Angle to Servo Length based on the physical mechanism. */
  private static Distance convertHoodAngleToServoLength(Angle hoodAngle) {
    Angle plateAngle = hoodAngle.plus(crankTangentToLaunchAngle);
    Angle internalTheta = mechanismTotalAngle.minus(plateAngle);

    double a = groundLinkDistance.in(Inches);
    double b = crankArmLength.in(Inches);
    double cosTheta = Math.cos(internalTheta.in(Radians));

    double servoLengthSquared = (a * a) + (b * b) - (2 * a * b * cosTheta);
    double servoLength = Math.sqrt(Math.max(0, servoLengthSquared));

    return Inches.of(
        MathUtil.clamp(servoLength, servoMinLength.in(Inches), servoMaxLength.in(Inches)));
  }

  /** Returns true if the flywheels are spun up and the hood has finished moving. */
  public boolean isReadyToShoot() {
    double errorRps =
        targetVelocity.in(RadiansPerSecond) - flywheelInputs.velocity.in(RadiansPerSecond);
    boolean flywheelReady = Math.abs(errorRps) <= flywheelAngularTolerance.in(RadiansPerSecond);
    boolean hoodReady = hoodInputs.atSetpoint;

    return flywheelReady && hoodReady && isShooting;
  }

  /** Safely stops the flywheels. */
  public void stop() {
    targetVelocity = RadiansPerSecond.of(0.0);
    flywheelIO.stop();
    isShooting = false;
  }
}
