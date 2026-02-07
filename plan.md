# Shooting Implementation Plan (Refined)

The objective is to implement a "Shooting Mode" that coordinates the drivetrain, flywheel, hood, and indexer.
**Key Constraints**:
1.  **Zone Restricted**: Shooting mode can only be entered/maintained while in a specific field zone.
2.  **Toggle Activation**: The driver toggles the mode on/off.
3.  **Hub Centric**: Rotation auto-aligns to the Hub Center (dependent on Alliance side).

## 1. Prerequisite Commands & Subsystems
- **Flywheel**: Expose velocity (`getVelocity()`).
- **Shooter**: Coordinator methods (`prepare`, `feed`, `isReady`).
- **Drive**: Expose `getPose()` for zone checks.

## 2. Field Logic
We need to define the geometry for the game.
- **Shooting Zone**: Rectangular region from (0.0, 0.0) to (4.015, 8.100) meters.
    - *Action*: Implement mirroring logic for Red alliance (flipping X coordinate based on field length).
- **Hub Location**:
    - **Blue Alliance**: x: 4.625, y: 4.035 (Meters).
    - **Red Alliance**: Mirror of Blue (Field Length - x, y).
    - *Action*: Create a `FieldConstants` class or utility to provide `getHubCenter()` dynamically based on valid alliance.

## 3. RobotContainer Logic

### Controls
- **Toggle Shooting Mode**: `Right Bumper` (Toggle).
- **Fire**: `Right Trigger` (Hold to feed).

### Structure

#### A. Zone Check
Helper boolean to check position commands:
```java
BooleanSupplier isInZone = () -> {
    return FieldConstants.shootingZone.contains(drive.getPose().getTranslation());
};
```

#### B. The "Shooting Mode" Command
This command runs while the mode is toggled ON.
1.  **Drive**: Use `joystickDriveAtAngle`.
    - *XY*: Driver controlled.
    - *Rotation*: Calculated angle to `FieldConstants.getHubCenter()`.
2.  **Shooter**: `prepareToShoot()`.
    - Revs flywheel to target speed.
    - Sets hood angle.
3.  **Interrupt**:
    - If `!isInZone()`, this command should cancel (automatically turning off the mode).

#### C. Bindings
```java
Command shootingModeCommand = Commands.parallel(
    // Drive + Aim
    DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        this::getHubRotation // Calculates angle to Hub
    ),
    // Rev Shooter
    shooter.prepareToShoot(...)
).until(() -> !isInZone.getAsBoolean()); // Safety: cancel if leaving zone

// Toggle binding with condition
controller.rightBumper()
    .and(isInZone) // Can only toggle ON if in zone
    .toggleOnTrue(shootingModeCommand);

// Firing Logic
new Trigger(shootingModeCommand::isScheduled) // Only if mode is active
    .and(controller.rightTrigger())           // And trigger pressed
    .and(shooter::isReady)                    // And mechanicals ready
    .and(atTargetRot)                         // And aimed
    .whileTrue(shooter.feed());
```

## Summary of Tasks
1. [ ] **FieldConstants**: Define Hub location and Shooting Zone boundary.
2. [ ] **Subsystems**: Update `Flywheel` (velocity), `Shooter` (logic).
3. [ ] **RobotContainer**: Implement `getHubRotation`, zone checks, and bindings.
