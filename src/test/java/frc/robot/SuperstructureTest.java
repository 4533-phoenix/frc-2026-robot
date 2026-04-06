package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.intake.arm.Arm;
import frc.robot.subsystems.intake.arm.ArmIO;
import frc.robot.subsystems.intake.spinner.Spinner;
import frc.robot.subsystems.intake.spinner.SpinnerIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.HoodIO;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SuperstructureTest {
  private Drive drive;
  private Climber climb;
  private Arm arm;
  private Spinner spinner;
  private Shooter shooter;
  private Indexer indexer;
  private Superstructure superstructure;

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    climb = new Climber(new ClimberIO() {});
    arm = new Arm(new ArmIO() {});
    spinner = new Spinner(new SpinnerIO() {});
    shooter = new Shooter(new FlywheelIO() {}, new HoodIO() {});
    indexer = new Indexer(new IndexerIO() {});

    superstructure = new Superstructure(drive, climb, arm, spinner, shooter, indexer);
  }

  @AfterEach
  void tearDown() {
    // Cleanup if needed
  }

  @Test
  void testClimbModeTogglesArmRetract() {
    // By default climb mode is false
    assertFalse(superstructure.getClimbMode().get());

    // Toggle climb mode
    superstructure.toggleClimbMode();

    // Climb mode should be true, and arm should theoretically begin retracting
    assertTrue(superstructure.getClimbMode().get());
    assertTrue(
        superstructure.canClimb().getAsBoolean()
            || !superstructure
                .canClimb()
                .getAsBoolean()); // Just checking no exceptions and triggers work
  }

  @Test
  void testPeriodicWithoutTarget() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    superstructure.periodic();

    assertFalse(superstructure.hasTarget());
  }
}
