// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.shooter.ShooterConstants;

// public class ShooterCommands {
//   private ShooterCommands() {}

//   public static Command runFlywheel(Shooter shooter) {
//     return Commands.runEnd(
//         () -> shooter.runFlywheel(ShooterConstants.defaultFlywheelVoltage),
//         shooter::stopFlywheel,
//         shooter);
//   }

//   public static Command runFlywheel(Shooter shooter, double volts) {
//     return Commands.runEnd(() -> shooter.runFlywheel(volts), shooter::stopFlywheel, shooter);
//   }

//   public static Command setHoodPosition(Shooter shooter, double position) {
//     return Commands.runOnce(() -> shooter.setHoodPosition(position), shooter);
//   }

//   public static Command aimHood(Shooter shooter) {
//     return setHoodPosition(shooter, ShooterConstants.hoodExtendPosition);
//   }

//   public static Command stowHood(Shooter shooter) {
//     return setHoodPosition(shooter, ShooterConstants.hoodRetractPosition);
//   }
// }
