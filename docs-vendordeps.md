# FRC Vendor Dependencies

## Core Libraries

### REV Robotics
- **REVLib (2024)**
  - URL: `https://software-metadata.revrobotics.com/REVLib-2024.json`
  - Used for: SPARK MAX motor controllers, NEO/NEO 550 motors, Through Bore Encoder, Color Sensor V3
  - Documentation: [REV Hardware Client](https://docs.revrobotics.com/hardware-client/)

### CTRE Phoenix (Cross The Road Electronics)
- **Phoenix 6 (2024)**
  - URL: `https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-latest.json`
  - Used for: Talon FX (Falcon 500), Pigeon 2.0, CANcoder, CANdle
  - Documentation: [Phoenix 6 Documentation](https://v6.docs.ctr-electronics.com/)
  
- **Phoenix 5 (Legacy)**
  - URL: `https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-latest.json`
  - Used for: Legacy devices (Talon SRX, Victor SPX, etc.)
  - Note: Consider migrating to Phoenix 6 for newer projects

### PathPlanner
- **PathPlanner (2024)**
  - URL: `https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json`
  - Used for: Path planning and autonomous routines
  - Documentation: [PathPlanner Documentation](https://pathplanner.dev/home.html)

### Kauai Labs navX
- **navX-MXP (2024)**
  - URL: `https://dev.studica.com/releases/2024/NavX.json`
  - Used for: navX-MXP and navX2-MXP IMUs
  - Documentation: [navX-MXP Guide](https://pdocs.kauailabs.com/navx-mxp/)

### PhotonVision
- **PhotonLib (2024)**
  - URL: `https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json`
  - Used for: Vision processing and AprilTag detection
  - Documentation: [PhotonVision Docs](https://docs.photonvision.org/)

### Limelight
- **limelightlib**
  - URL: `https://downloads.limelightvision.io/limelightlib-json/limelightlib.json`
  - Used for: Limelight camera integration
  - Documentation: [Limelight Docs](https://docs.limelightvision.io/)

## Additional Libraries

### Playing With Fusion
- **PWF Lib (2024)**
  - URL: `https://www.playingwithfusion.com/frc/playingwithfusion2024.json`
  - Used for: Venom motor controllers, TimeOfFlight sensors

### Swerve Drive Specialties
- **SDS Lib**
  - URL: `https://raw.githubusercontent.com/SwerveDriveSpecialties/swerve-lib/master/SdsSwerveLib.json`
  - Used for: SDS MK3/MK4/MK4i swerve modules

### Team 364 BananaLib
- **BananaLib (Swerve)**
  - URL: `https://raw.githubusercontent.com/FRC364/BananaLib/master/BananaLib.json`
  - Used for: Alternate swerve drive implementation

## Installation Instructions

1. Open your project in VS Code
2. Press Ctrl+Shift+P (Cmd+Shift+P on Mac)
3. Type "WPILib: Manage Vendor Libraries"
4. Select "Install new libraries (online)"
5. Paste the desired vendor URL
6. Click "Yes" to apply changes
7. Build project to download dependencies

## Recommended Dependencies For Common Hardware

### Swerve Drive
- REVLib (for SPARK MAX)
- Phoenix 6 (for CANcoder)
- NavX or Phoenix 6 (for IMU)
- Optional: SDS Lib or BananaLib

### Vision Processing
- PhotonLib (general vision)
- Limelight Lib (if using Limelight)

### Motor Control
- REVLib (for NEO/NEO 550 and SPARK MAX)
- Phoenix 6 (for Falcon 500)
- Phoenix 5 (only if using older CTRE hardware)

## Notes
- Always check for the latest vendor URLs as they may update between seasons
- Some vendors may release mid-season updates
- Test thoroughly after updating vendor dependencies
- Consider impact on deployment size when adding libraries
- Keep dependencies minimal to reduce potential conflicts