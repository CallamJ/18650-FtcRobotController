# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a FIRST Tech Challenge (FTC) robotics project for the INTO THE DEEP (2024-2025) competition season. The codebase consists of:
- **FtcRobotController**: Official FTC SDK library code (avoid modifying)
- **TeamCode**: Custom team robot code (primary development area)

## Build Requirements and Commands

### Prerequisites
- **Java 11 or newer** required (build fails with Java 8)
- **Android Studio Ladybug (2024.2)** or later for development
- **Android SDK 30** for compilation

### Common Commands
```bash
# Build the project
./gradlew build

# Build only TeamCode module
./gradlew :TeamCode:build

# Clean build artifacts
./gradlew clean

# Install APK to connected robot controller
./gradlew installDebug

# List all available gradle tasks
./gradlew tasks
```

## Architecture

### TeamCode Structure

The TeamCode module follows a layered architecture:

1. **Core Layer** (`teamcode/core/`)
   - `OpModeCore`: Abstract base class for all OpModes, manages component lifecycle
   - `BasicOpModeCore`: Basic OpMode functionality
   - `TeleOpCore`: Base for driver-controlled OpModes
   - `AutonomousCore`: Base for autonomous OpModes
   - All OpModes extend from these core classes

2. **Hardware Layer** (`teamcode/hardware/`)
   - `Hardware`: Static hardware accessor with caching
   - `SmartMotor`, `SmartServo`, `SmartSensor`: Hardware wrappers with error handling
   - `controllers/`: PID and control algorithms
   - `filters/`: Data filtering (e.g., RollingAverage)
   - Hardware is accessed via `Hardware.getMotor()`, `Hardware.getServo()`, etc.

3. **Components Layer** (`teamcode/components/`)
   - High-level robot subsystems:
     - `DriveBase`: Mecanum drive system
     - `TelescopingArm`: Extension mechanism
     - `Grip`, `PitchWrist`, `RollWrist`: Manipulator components
     - `TiltBase`: Base tilting mechanism
   - Components are initialized in `OpModeCore.initialize()`

4. **Vision Layer** (`teamcode/vision/`)
   - `AprilTagReader`: AprilTag detection
   - `MultiAprilTagReader`: Multiple tag handling
   - Vision processing integration

5. **Autonomous Navigation** (`teamcode/drive/`)
   - RoadRunner integration for path planning
   - `ConfiguredMecanumDrive`: Configured drive for RoadRunner
   - Trajectory sequences and tuning utilities

### Key Design Patterns

1. **Singleton Pattern**: `OpModeCore.getInstance()` provides global access
2. **Hardware Caching**: `HardwareCache` prevents repeated hardware lookups
3. **Component Initialization**: Components initialized once in `OpModeCore`
4. **Static Hardware Access**: Hardware devices accessed through static `Hardware` class

### OpMode Development

New OpModes should:
1. Extend appropriate core class (`TeleOpCore` or `AutonomousCore`)
2. Implement required abstract methods
3. Use `@TeleOp` or `@Autonomous` annotations
4. Access hardware through `Hardware` class
5. Use existing components from `OpModeCore`

Example structure:
```java
@TeleOp(name="MyTeleOp", group="Linear OpMode")
public class MyTeleOp extends TeleOpCore {
    @Override
    protected void tick() {
        // Control logic here
        driveBase.drive(gamepad1);
    }
}
```

## Important Notes

### FTC vs TeamCode Separation
- **Never modify** code in `FtcRobotController/` - it's official FTC SDK code
- **All custom code** goes in `TeamCode/` module
- To suppress FTC library warnings, configure `lintOptions` in TeamCode's build.gradle

### Hardware Configuration
- Robot configuration defined in Robot Controller app
- Device names in code must match configuration exactly
- Hardware initialization happens through `hardwareMap` in OpModes

### Testing and Deployment
- OpModes are deployed to the Robot Controller phone via USB or WiFi
- Use `@Disabled` annotation to hide OpModes from driver station
- FTC Dashboard available at `http://192.168.43.1:8080/dash` when connected

### Competition Rules
- OpModes must complete initialization within 30 seconds
- Autonomous period is 30 seconds
- TeleOp period is 2 minutes

## Development Workflow

1. Create/modify OpModes in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
2. Build using gradle or Android Studio
3. Deploy to Robot Controller
4. Select and run OpMode from Driver Station app
5. Monitor telemetry and logs via Driver Station or FTC Dashboard