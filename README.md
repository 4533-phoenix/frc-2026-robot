<h1 align="left">
FRC 2026 Robot Code
<img src="https://phoenix4533.org/favicon.svg" align="right" width="45" height="45" />
</h1>

Welcome to the official repository for **FRC Team 4533 Phoenix's 2026 Robot Code**! This repository contains all the code for our robot, including subsystems, commands, and autonomous routines.

## Documentation

Comprehensive documentation for this project is available at [FRC 2026 Robot Docs](https://4533-phoenix.github.io/frc-2026-robot/).

## Vision System Overview

Our robot uses a robust vision system to localize on the field with AprilTags and other camera-based methods.

**How vision works:**

- The robot code connects to a coprocessor running [chalkydri](https://github.com/chalkydri/chalkydri), a Rust-based vision solution developed by our team. Chalkydri processes camera feeds on the coprocessor (e.g., Raspberry Pi, Jetson) and sends vision measurements (poses, uncertainties, timestamps, tag detections) to the robot over the network.
- On the robot, native C code (see `src/main/native/c/vision_server.c`) implements a fast, lock-free ring buffer and JNI interface for receiving vision data from the coprocessor. This C code packs observations and exposes them efficiently to the Java code.
- The Java-side subsystem (`Vision.java`) reads these measurements, filters/tag-counts, and integrates them into the robot's pose estimator. If cameras or coprocessor communication is lost, the system issues alerts to the drivers.
- All vision measurements are temporally aligned and uncertainty-weighted so robot pose is updated only with quality data.

**Integration benefits:**
- High reliability and speed via Rust and C for vision data ingestion
- Modular: you can swap coprocessors or vision algorithms with minimal changes to robot code
- All vision logs/status are recorded for debugging and performance analysis

_For details on Chalkydri, visit the [project repository](https://github.com/chalkydri/chalkydri)._

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

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contact

For questions or support, please contact us via our [team website](https://phoenix4533.org/).

---
*Built with ❤️ by FRC Team 4533 Phoenix.*
