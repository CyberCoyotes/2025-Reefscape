# FRC Third-Party Software Guide

## Essential Development Tools

### WPILib Suite
- **VS Code with WPILib**
  - Download: [WPILib Releases](https://github.com/wpilibsuite/allwpilib/releases)
  - Includes: VS Code, FRC extension, Java/C++ tools
  - Required for: Robot programming

### Hardware Configuration

- **REV Hardware Client**
  - Download: [REV Robotics Client](https://docs.revrobotics.com/rev-hardware-client/)
  - Used for: Configuring SPARK MAX, NEO motors
  - Features: Motor testing, firmware updates

- **Phoenix Tuner X**
  - Download: [CTRE Phoenix Tuner X](https://apps.microsoft.com/store/detail/phoenix-tuner-x/9NVV4PWDW27Z)
  - Used for: CTRE device configuration (Falcon 500, CANcoder, etc.)
  - Features: Device configuration, firmware updates, real-time plotting

- **Limelight Application**
  - Download: [Limelight Releases](https://limelightvision.io/pages/downloads)
  - Used for: Limelight camera configuration
  - Features: Vision pipeline setup, calibration

### Robot Control & Testing

- **FRC Driver Station**
  - Download: [NI FRC Game Tools](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html)
  - Required for: Robot control
  - Note: Must match current FRC season

- **FRC Dashboard**
  - **Glass**
    - Included with WPILib
    - Used for: Data visualization, subsystem testing
  - **Shuffleboard**
    - Included with WPILib
    - Used for: Custom dashboard layouts, data visualization
  - **AdvantageScope**
    - Download: [AdvantageScope Releases](https://github.com/Mechanical-Advantage/AdvantageScope/releases)
    - Used for: Advanced data logging, visualization
    - Features: 3D field view, plotting, log analysis

### Path Planning & Autonomous

- **PathPlanner**
  - Download: [PathPlanner Releases](https://github.com/mjansen4857/pathplanner/releases)
  - Used for: Creating autonomous paths
  - Features: Path generation, event markers, visualization

- **Choreo**
  - Download: [Choreo](https://choreo.firstinspires.org/)
  - Used for: Advanced path planning
  - Features: Swerve-specific paths, constraints

### Vision Tools

- **PhotonVision**
  - Download: [PhotonVision Releases](https://docs.photonvision.org/en/latest/docs/installation/installation.html)
  - Used for: Vision processing, AprilTag detection
  - Features: Camera calibration, pipeline configuration

### CAD & Design

- **OnShape**
  - Access: [OnShape for FRC](https://www.onshape.com/en/education/frc)
  - Used for: Robot CAD, design
  - Features: Free FRC team access

## Optional But Useful

### Development Tools

- **GitHub Desktop**
  - Download: [GitHub Desktop](https://desktop.github.com/)
  - Used for: Git version control
  - Features: Visual git interface

- **Gradle**
  - Download: [Gradle Releases](https://gradle.org/releases/)
  - Used for: Build automation
  - Note: Included with WPILib, separate install optional

### Testing & Analysis

- **OutlineViewer**
  - Included with WPILib
  - Used for: NetworkTables debugging
  - Features: Real-time NT viewing/editing

- **REV Robot Characterization**
  - Download: [REV Robotics Tools](https://docs.revrobotics.com/sparkmax/software-resources/spark-max-characterization)
  - Used for: Motor/mechanism characterization
  - Features: Auto-generated feedforward values

## Installation Notes

1. Install WPILib first as it's required for most other tools
2. Install NI FRC Game Tools before connecting to a robot
3. Some tools may require admin privileges to install
4. Keep tools updated throughout the season
5. Check compatibility with current FRC season

## Device Firmware Resources

- CTRE Phoenix Framework: [Link](https://store.ctr-electronics.com/software/)
- REV Hardware Client: [Link](https://docs.revrobotics.com/rev-hardware-client/)
- NavX MXP: [Link](https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/)
- Limelight: [Link](https://docs.limelightvision.io/en/latest/getting_started.html#imaging)