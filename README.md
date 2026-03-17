<h1 align="left">
FRC 2026 Robot Code
<img src="https://phoenix4533.org/favicon.svg" align="right" width="45" height="45" />
</h1>

Welcome to the official repository for **FRC Team 4533 Phoenix's 2026 Robot Code**! This repository contains all the code for our robot, including subsystems, commands, and autonomous routines.

## Documentation

We maintain two primary sources of documentation:

- **[The Wiki](https://github.com/4533-Phoenix/frc-2026-robot/wiki):** Comprehensive guides on our architecture, custom native JNI libraries (Whacknet & SparkTap), high-frequency odometry, superstructure coordination, and core coding philosophies. **Start here if you are new to the codebase!**
- **[Javadocs](https://4533-phoenix.github.io/frc-2026-robot/):** Auto-generated API documentation for all classes and methods.

## Advanced Features & Optimizations

Our codebase goes far beyond a standard WPILib implementation. To maximize performance, minimize Garbage Collector (GC) lag spikes, and ensure absolute safety, we've built several custom integrations:

- **[Whacknet: Zero-Allocation Vision](https://github.com/4533-Phoenix/frc-2026-robot/wiki/Whacknet:-UDP-Vision-Protocol):** A custom C/JNI UDP server that parses vision data from our Rust coprocessor ([Chalkydri](https://github.com/chalkydri/chalkydri)). It uses a lock-free ring buffer, kernel-level timestamps, and the Java Flyweight pattern to achieve ultra-low latency with zero memory allocation.
- **[SparkTap: CAN Interception](https://github.com/4533-Phoenix/frc-2026-robot/wiki/SparkTap:-CAN-Interception):** A C++ native library that reads CAN frames directly via WPILib HAL stream sessions into a shared memory map. This allows Java to read motor telemetry in sub-milliseconds using atomics, completely eliminating JNI overhead.
- **[High-Frequency Odometry & Dual Gyros](https://github.com/4533-Phoenix/frc-2026-robot/wiki/Drivetrain-&-Odometry):** A dedicated 200Hz background thread strictly for polling drive kinematics. We also use a dual-gyro setup (NavX + Canandgyro) that automatically compensates for drift when the robot is stationary.
- **[Superstructure Orchestration](https://github.com/4533-Phoenix/frc-2026-robot/wiki/Superstructure-Coordination):** Instead of messy subsystem-to-subsystem communication, a centralized state machine acts as a gatekeeper. It safely manages physical interlocks, aiming algorithms (like two-pass lead compensation), and translates driver intent into hardware actions.


## Team Website

Learn more about **FRC Team 4533 Phoenix** by visiting our [official website](https://phoenix4533.org/).

## Getting Started

### Prerequisites

- [WPILib](https://docs.wpilib.org/en/stable/index.html)
- Java 17
- Gradle 8.0 or higher
- A compatible IDE (e.g., VS Code with WPILib extension)

### Cloning the Repository

```bash
git clone https://github.com/4533-Phoenix/frc-2026-robot.git
cd frc-2026-robot
```

### Building and Deploying

1. Open the project in your IDE.
2. Connect to the robot's network.
3. Use the WPILib extension to build and deploy the code.

## Contributing

We welcome contributions from team members and the community! To contribute:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Ensure all workflows pass successfully before submitting your changes.
4. Document all new code thoroughly, including Javadoc comments for methods, classes, and fields.
5. Submit a pull request with a detailed description of your changes.

Pull requests that do not meet the documentation or workflow requirements will not be accepted.

## License

This project is licensed under the BSD License. See the [LICENSE](LICENSE) file for details.

## Contact

For questions or support, please contact us via our [team website](https://phoenix4533.org/).

## Disclaimer

This code was designed to push the boundaries for what the RoboRIO can *really* do to the best of my abilities. Feel free to take inspiration and/or copy from this codebase. Think of it as a send-off of the RoboRIO hardware and the end of my FRC career. Here be dragons.

---
*Built with ❤️ by FRC Team 4533 Phoenix.*
