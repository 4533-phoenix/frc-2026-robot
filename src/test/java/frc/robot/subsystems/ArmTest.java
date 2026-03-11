// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.intake.arm.Arm;
import frc.robot.subsystems.intake.arm.ArmConstants;
import frc.robot.subsystems.intake.arm.ArmIO;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class ArmTest {

  @BeforeAll
  public static void setup() {
    assert HAL.initialize(500, 0); // Prevent Timer/Alert crashes
  }

  // Create a dummy IO that lets us inject fake sensor data
  private static class DummyArmIO implements ArmIO {
    public ArmIOInputs inputsToInject = new ArmIOInputs();
    public Angle lastCommandedAngle = null;

    @Override
    public void updateInputs(ArmIOInputs inputs) {
      inputs.position = inputsToInject.position;
    }

    @Override
    public void setPosition(Angle angle) {
      lastCommandedAngle = angle;
    }
  }

  @Test
  public void testArmTriggersAndGoals() {
    DummyArmIO dummyIO = new DummyArmIO();
    Arm arm = new Arm(dummyIO);

    // Initial state: robot turns on, arm is somewhere random.
    dummyIO.inputsToInject.position = Degrees.of(90.0);
    arm.periodic();
    assertFalse(arm.isDeployed().getAsBoolean());
    assertFalse(arm.isRetracted().getAsBoolean());

    // Send the "Deploy" command
    arm.deploy().initialize();
    arm.periodic();

    // Ensure the subsystem sent the deploy position to the hardware
    assertEquals(
        ArmConstants.DEPLOYED_POSITION.in(Degrees), dummyIO.lastCommandedAngle.in(Degrees), 1e-6);

    // Simulate the physical arm reaching the target position
    dummyIO.inputsToInject.position = ArmConstants.DEPLOYED_POSITION;
    arm.periodic();

    // The trigger should now fire
    assertTrue(arm.isDeployed().getAsBoolean(), "Arm should report as deployed");
  }
}
