// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.lowlevel;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import org.junit.jupiter.api.Test;

public class WhacknetTest {

  @Test
  public void testPacketViewOffsets() throws Exception {
    // Create a dummy buffer simulating the C++ shared memory
    Field field = Whacknet.class.getDeclaredField("STRUCT_SIZE");
    field.setAccessible(true);
    int structSize = (Integer) field.get(null);
    ByteBuffer buffer = ByteBuffer.allocateDirect(structSize);
    buffer.order(ByteOrder.nativeOrder());

    // Write specific values at the expected offsets
    buffer.putDouble(0, 1.23); // X
    buffer.putDouble(8, 4.56); // Y
    buffer.putDouble(16, 7.89); // Z
    buffer.putDouble(24, 0.1); // Roll
    buffer.putDouble(32, 0.2); // Pitch
    buffer.putDouble(40, 3.14); // Yaw
    buffer.putDouble(48, 0.01); // Std X
    buffer.putDouble(56, 0.02); // Std Y
    buffer.putDouble(64, 0.03); // Std Rot
    buffer.putLong(72, 123456789L); // Timestamp
    buffer.put(80, (byte) 2); // Camera ID
    buffer.put(81, (byte) 3); // Num Tags

    // Instantiate Whacknet (will fail safely if not on real robot, but we can use reflection
    // to test the PacketView class directly without loading JNI)
    Whacknet whacknet = Whacknet.getInstance();

    // Inject our fake buffer using reflection for testing
    Field bufferField = Whacknet.class.getDeclaredField("readBuffer");
    bufferField.setAccessible(true);
    bufferField.set(whacknet, buffer);

    Whacknet.PacketView view = whacknet.new PacketView();
    view.setIndex(0); // Point to our fake packet

    // Assert that the offsets read exactly what we wrote
    assertEquals(1.23, view.getX(), 1e-6, "X offset mismatch");
    assertEquals(4.56, view.getY(), 1e-6, "Y offset mismatch");
    assertEquals(7.89, view.getZ(), 1e-6, "Z offset mismatch");
    assertEquals(0.1, view.getRoll(), 1e-6, "Roll offset mismatch");
    assertEquals(0.2, view.getPitch(), 1e-6, "Pitch offset mismatch");
    assertEquals(3.14, view.getYaw(), 1e-6, "Yaw offset mismatch");
    assertEquals(0.01, view.getStdX(), 1e-6, "StdX offset mismatch");
    assertEquals(123456789L, view.getTimestamp(), "Timestamp offset mismatch");
    assertEquals(2, view.getCameraId(), "Camera ID offset mismatch");
    assertEquals(3, view.getNumTags(), "Num tags offset mismatch");
  }
}
