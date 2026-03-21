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
    ByteBuffer buffer = ByteBuffer.allocateDirect(Whacknet.STRUCT_SIZE);
    buffer.order(ByteOrder.nativeOrder());

    // Write specific values at the expected offsets
    buffer.putDouble(0, 1.23); // X
    buffer.putDouble(8, 4.56); // Y
    buffer.putDouble(16, 3.14); // Rot
    buffer.putDouble(24, 0.01); // Std X
    buffer.putDouble(32, 0.02); // Std Y
    buffer.putDouble(40, 0.03); // Std Rot
    buffer.putLong(48, 123456789L); // Timestamp
    buffer.put(56, (byte) 2); // Camera ID
    buffer.put(57, (byte) 3); // Num Tags

    // Instantiate Whacknet (will fail safely if not on real robot, but we can use reflection
    // to test the PacketView class directly without loading JNI)
    Whacknet whacknet = Whacknet.getInstance();

    // Inject our fake buffer using reflection for testing
    Field bufferField = Whacknet.class.getDeclaredField("buffer");
    bufferField.setAccessible(true);
    bufferField.set(whacknet, buffer);

    Whacknet.PacketView view = whacknet.new PacketView();
    view.setIndex(0); // Point to our fake packet

    // Assert that the offsets read exactly what we wrote
    assertEquals(1.23, view.getX(), 1e-6, "X offset mismatch");
    assertEquals(4.56, view.getY(), 1e-6, "Y offset mismatch");
    assertEquals(3.14, view.getRot(), 1e-6, "Rot offset mismatch");
    assertEquals(0.01, view.getStdX(), 1e-6, "StdX offset mismatch");
    assertEquals(123456789L, view.getTimestamp(), "Timestamp offset mismatch");
    assertEquals(2, view.getCameraId(), "Camera ID offset mismatch");
    assertEquals(3, view.getNumTags(), "Num tags offset mismatch");
  }
}
