// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.lowlevel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.ClosedChannelException;
import java.nio.channels.DatagramChannel;
import java.util.function.ObjIntConsumer;
import java.util.function.Supplier;

/**
 * Pure Java implementation of the Whacknet UDP Vision Protocol.
 *
 * <p>Handles high-frequency vision data from coprocessors and broadcasts robot telemetry. Uses
 * Direct ByteBuffers and the Flyweight pattern to achieve zero-allocation performance, avoiding
 * Garbage Collector pressure during matches.
 */
public class Whacknet implements AutoCloseable {
  /** A record representing the robot's telemetry data. */
  public record RobotTelemetry(
      long timestampUs,
      Angle roll,
      Angle pitch,
      Angle yaw,
      AngularVelocity rollVel,
      AngularVelocity pitchVel,
      AngularVelocity yawVel) {}

  private static Whacknet instance;

  // Constants
  private static final int MAX_QUEUE_SIZE = 64;
  private static final int MASK = MAX_QUEUE_SIZE - 1;
  private static final int STRUCT_SIZE = 64;
  private static final int TELEMETRY_SIZE = 64;

  // Struct Offsets (VisionMeasurement)
  private static final int OFFSET_X = 0;
  private static final int OFFSET_Y = 8;
  private static final int OFFSET_ROT = 16;
  private static final int OFFSET_STD_X = 24;
  private static final int OFFSET_STD_Y = 32;
  private static final int OFFSET_STD_ROT = 40;
  private static final int OFFSET_TIMESTAMP = 48;
  private static final int OFFSET_CAMERA_ID = 56;
  private static final int OFFSET_NUM_TAGS = 57;

  // Members
  private final ByteBuffer queueBuffer;
  private final ByteBuffer readBuffer;
  private final PacketView packetView = new PacketView();

  // Thread-safe indices for the circular buffer
  private volatile int head = 0;
  private volatile int tail = 0;

  private DatagramChannel receiverChannel;
  private DatagramChannel senderChannel;
  private InetSocketAddress broadcastAddress;
  private Thread receiverThread;
  private int currentPacketCount = 0;
  private boolean isRunning = false;
  private Notifier notifier;

  /**
   * Returns the singleton instance of Whacknet.
   *
   * @return The Whacknet instance.
   */
  public static synchronized Whacknet getInstance() {
    if (instance == null) {
      instance = new Whacknet();
    }
    return instance;
  }

  private Whacknet() {
    // Allocate off-heap memory to prevent GC overhead
    queueBuffer = ByteBuffer.allocateDirect(MAX_QUEUE_SIZE * STRUCT_SIZE);
    queueBuffer.order(ByteOrder.nativeOrder());

    readBuffer = ByteBuffer.allocateDirect(MAX_QUEUE_SIZE * STRUCT_SIZE);
    readBuffer.order(ByteOrder.nativeOrder());

    // Report custom framework usage to HAL
    HAL.report(tResourceType.kResourceType_Framework, 4533);

    // Register a shutdown hook to ensure resources are freed on exit
    Runtime.getRuntime().addShutdownHook(new Thread(this::close));
  }

  /**
   * Starts the vision server to receive observations from coprocessors.
   *
   * @param port The UDP port to listen on.
   */
  public void startServer(int port) {
    if (isRunning) return;

    try {
      receiverChannel = DatagramChannel.open();
      receiverChannel.bind(new InetSocketAddress(port));
      receiverChannel.setOption(StandardSocketOptions.SO_RCVBUF, 4 * 1024 * 1024); // 4MB buffer
      receiverChannel.configureBlocking(true);

      isRunning = true;
      receiverThread = new Thread(this::receiverLoop, "WhacknetReceiver");
      receiverThread.setDaemon(true);
      receiverThread.setPriority(Thread.MAX_PRIORITY);
      receiverThread.start();

      System.out.println("[Whacknet-Java] Vision server started on port " + port);
    } catch (IOException e) {
      DriverStation.reportError("[Whacknet-Java] Failed to start server: " + e.getMessage(), true);
    }
  }

  /** Background thread for pulling UDP packets into the lock-free ring buffer. */
  private void receiverLoop() {
    ByteBuffer socketBuffer = ByteBuffer.allocateDirect(STRUCT_SIZE);
    socketBuffer.order(ByteOrder.nativeOrder());

    while (isRunning) {
      try {
        socketBuffer.clear();
        receiverChannel.receive(socketBuffer);
        socketBuffer.flip();

        if (socketBuffer.remaining() != STRUCT_SIZE) continue;

        int h = head;
        int t = tail;
        int nextH = (h + 1) & MASK;

        // If queue is not full, copy packet into shared memory
        if (nextH != t) {
          int offset = h * STRUCT_SIZE;
          // Thread-safe copy to the direct buffer
          queueBuffer.limit(offset + STRUCT_SIZE);
          queueBuffer.position(offset);
          queueBuffer.put(socketBuffer);
          head = nextH; // Atomic update
        }
      } catch (ClosedChannelException e) {
        System.out.println("[Whacknet-Java] Receiver thread stopping cleanly.");
        break;
      } catch (IOException e) {
        if (isRunning) {
          DriverStation.reportError(
              "[Whacknet-Java] Receiver loop error: " + e.getMessage(), false);
        }
      }
    }
  }

  /**
   * Broadcasts robot telemetry to the vision coprocessor for constrained pose solving.
   *
   * @param timestamp FPGA timestamp in microseconds.
   * @param roll Robot roll in radians.
   * @param pitch Robot pitch in radians.
   * @param yaw Robot yaw in radians.
   * @param rollVel Roll velocity in radians per second.
   * @param pitchVel Pitch velocity in radians per second.
   * @param yawVel Yaw velocity in radians per second.
   * @param port The UDP port to broadcast on.
   */
  public void broadcastTelemetry(
      long timestamp,
      double roll,
      double pitch,
      double yaw,
      double rollVel,
      double pitchVel,
      double yawVel,
      int port) {

    try {
      if (senderChannel == null) {
        senderChannel = DatagramChannel.open();
        senderChannel.setOption(StandardSocketOptions.SO_BROADCAST, true);
        senderChannel.configureBlocking(false);
        broadcastAddress = new InetSocketAddress("255.255.255.255", port);
        System.out.println("[Whacknet-Java] Broadcast channel initialized on port " + port);
      }

      // Re-use a single direct buffer for broadcasting to remain 0-allocation
      ByteBuffer telBuf = ByteBuffer.allocateDirect(TELEMETRY_SIZE);
      telBuf.order(ByteOrder.nativeOrder());
      telBuf.putLong(timestamp);
      telBuf.putDouble(roll);
      telBuf.putDouble(pitch);
      telBuf.putDouble(yaw);
      telBuf.putDouble(rollVel);
      telBuf.putDouble(pitchVel);
      telBuf.putDouble(yawVel);
      telBuf.flip();

      senderChannel.send(telBuf, broadcastAddress);

    } catch (IOException e) {
      DriverStation.reportError("[Whacknet-Java] Broadcast failed: " + e.getMessage(), false);
    }
  }

  /**
   * Registers a service to broadcast robot telemetry at a specified frequency. Clears the old
   * notifier if it exists. If the consumer is not null, it will be used to process the telemetry data.
   *
   * @param port The UDP port to broadcast on.
   * @param frequency The frequency at which to broadcast.
   * @param supplier A supplier for obtaining robot telemetry data.
   */
  public void registerTelemetryService(
      int port, Frequency frequency, Supplier<RobotTelemetry> supplier) {
    if (notifier != null) {
      notifier.stop();
    }

    notifier =
        new Notifier(
            () -> {
              RobotTelemetry data = supplier.get();
              if (data != null) {
                this.broadcastTelemetry(
                    data.timestampUs(),
                    data.roll().in(Radians),
                    data.pitch().in(Radians),
                    data.yaw().in(Radians),
                    data.rollVel().in(RadiansPerSecond),
                    data.pitchVel().in(RadiansPerSecond),
                    data.yawVel().in(RadiansPerSecond),
                    port);
              }
            });
    notifier.startPeriodic(1.0 / frequency.in(Hertz));
  }

  /**
   * Drains the internal ring buffer into a snapshot buffer for the main loop to process.
   *
   * @return The number of packets available to process this frame.
   */
  public int readPackets() {
    int h = head;
    int t = tail;
    int count = 0;

    while (t != h && count < MAX_QUEUE_SIZE) {
      // Copy from shared ring buffer to the main-thread-safe read buffer
      int srcOffset = t * STRUCT_SIZE;
      int dstOffset = count * STRUCT_SIZE;

      for (int i = 0; i < STRUCT_SIZE; i++) {
        readBuffer.put(dstOffset + i, queueBuffer.get(srcOffset + i));
      }

      t = (t + 1) & MASK;
      count++;
    }

    tail = t; // Update tail so receiver thread knows we've freed up space
    currentPacketCount = count;
    return count;
  }

  /**
   * Gracefully shuts down the Whacknet server, freeing UDP ports and stopping the receiver thread.
   */
  @Override
  public synchronized void close() {
    if (!isRunning) return;
    isRunning = false;

    try {
      if (receiverChannel != null && receiverChannel.isOpen()) {
        receiverChannel.close();
      }
      if (senderChannel != null && senderChannel.isOpen()) {
        senderChannel.close();
      }

      if (receiverThread != null && receiverThread.isAlive()) {
        receiverThread.interrupt();
      }

      System.out.println("[Whacknet-Java] Sockets closed and resources freed.");
    } catch (IOException e) {
      System.err.println("[Whacknet-Java] Error during shutdown: " + e.getMessage());
    }
  }

  /**
   * Iterates over packets received since the last call to {@link #readPackets()}.
   *
   * @param consumer A lambda to process each packet view.
   */
  public void forEachPacket(ObjIntConsumer<PacketView> consumer) {
    for (int i = 0; i < currentPacketCount; i++) {
      packetView.setIndex(i);
      consumer.accept(packetView, i);
    }
  }

  /** Zero-allocation view into the ByteBuffer representing a single Vision Observation. */
  public class PacketView {
    private int baseOffset = 0;

    // Package-private so only Whacknet can move the cursor
    void setIndex(int index) {
      this.baseOffset = index * STRUCT_SIZE;
    }

    /**
     * @return X position in meters.
     */
    public double getX() {
      return readBuffer.getDouble(baseOffset + OFFSET_X);
    }

    /**
     * @return Y position in meters.
     */
    public double getY() {
      return readBuffer.getDouble(baseOffset + OFFSET_Y);
    }

    /**
     * @return Rotation in radians.
     */
    public double getRot() {
      return readBuffer.getDouble(baseOffset + OFFSET_ROT);
    }

    /**
     * @return Standard deviation of X position in meters.
     */
    public double getStdX() {
      return readBuffer.getDouble(baseOffset + OFFSET_STD_X);
    }

    /**
     * @return Standard deviation of Y position in meters.
     */
    public double getStdY() {
      return readBuffer.getDouble(baseOffset + OFFSET_STD_Y);
    }

    /**
     * @return Standard deviation of rotation in radians.
     */
    public double getStdRot() {
      return readBuffer.getDouble(baseOffset + OFFSET_STD_ROT);
    }

    /**
     * @return Timestamp in microseconds.
     */
    public long getTimestamp() {
      return readBuffer.getLong(baseOffset + OFFSET_TIMESTAMP);
    }

    /**
     * @return Camera ID.
     */
    public int getCameraId() {
      return Byte.toUnsignedInt(readBuffer.get(baseOffset + OFFSET_CAMERA_ID));
    }

    /**
     * @return Number of tags detected.
     */
    public int getNumTags() {
      return Byte.toUnsignedInt(readBuffer.get(baseOffset + OFFSET_NUM_TAGS));
    }

    /**
     * @return The pose as a Pose2d object.
     */
    public Pose2d getPose2d() {
      return new Pose2d(getX(), getY(), Rotation2d.fromRadians(getRot()));
    }
  }
}
