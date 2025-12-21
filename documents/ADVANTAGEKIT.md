# AdvantageKit API Documentation

## Overview
AdvantageKit is a comprehensive logging and replay framework developed by Team 6328 (Mechanical Advantage) for FRC robots. It provides structured data logging, replay capabilities, and advanced debugging tools.

## Version Information
- **Library Version**: 4.1.2
- **Java Group ID**: org.littletonrobotics.akit
- **Artifact**: akit-java
- **Maven URL**: https://frcmaven.wpi.edu/artifactory/littletonrobotics-mvn-release/

## Core Concepts

### LoggedRobot
AdvantageKit extends WPILib's TimedRobot with LoggedRobot, which provides automatic logging capabilities.

```java
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
    @Override
    public void robotInit() {
        // Initialize logging framework
    }
}
```

### IO Layer Pattern
AdvantageKit promotes an IO layer pattern that separates hardware interfaces from business logic, enabling comprehensive simulation and testing.

#### IO Interface Definition
```java
public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public double leftVelocityRPM = 0.0;
        public double rightVelocityRPM = 0.0;
        public double leftPositionRot = 0.0;
        public double rightPositionRot = 0.0;
        public double leftAppliedVolts = 0.0;
        public double rightAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        public double rightCurrentAmps = 0.0;
    }
    
    public default void updateInputs(DriveIOInputs inputs) {}
    
    public default void setVoltage(double leftVolts, double rightVolts) {}
}
```

#### Hardware Implementation
```java
public class DriveIOTalonFX implements DriveIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    
    public DriveIOTalonFX() {
        leftMotor = new TalonFX(1);
        rightMotor = new TalonFX(2);
    }
    
    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftVelocityRPM = leftMotor.getVelocity().getValueAsDouble() * 60.0;
        inputs.rightVelocityRPM = rightMotor.getVelocity().getValueAsDouble() * 60.0;
        inputs.leftPositionRot = leftMotor.getPosition().getValueAsDouble();
        inputs.rightPositionRot = rightMotor.getPosition().getValueAsDouble();
        inputs.leftAppliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightAppliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftCurrentAmps = leftMotor.getStatorCurrent().getValueAsDouble();
        inputs.rightCurrentAmps = rightMotor.getStatorCurrent().getValueAsDouble();
    }
    
    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftMotor.setControl(new VoltageOut(leftVolts));
        rightMotor.setControl(new VoltageOut(rightVolts));
    }
}
```

#### Simulation Implementation
```java
public class DriveIOSim implements DriveIO {
    private final DCMotorSim leftSim;
    private final DCMotorSim rightSim;
    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    
    public DriveIOSim() {
        leftSim = new DCMotorSim(LinearSystemId.identifyDrivetrainSystem(
            kvLinear, kaLinear, kvAngular, kaAngular), DCMotor.getCIM(2));
        rightSim = new DCMotorSim(LinearSystemId.identifyDrivetrainSystem(
            kvLinear, kaLinear, kvAngular, kaAngular), DCMotor.getCIM(2));
    }
    
    @Override
    public void updateInputs(DriveIOInputs inputs) {
        leftSim.update(0.02);
        rightSim.update(0.02);
        
        inputs.leftVelocityRPM = leftSim.getAngularVelocityRPM();
        inputs.rightVelocityRPM = rightSim.getAngularVelocityRPM();
        inputs.leftPositionRot = leftSim.getAngularPositionRotations();
        inputs.rightPositionRot = rightSim.getAngularPositionRotations();
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.leftCurrentAmps = leftSim.getCurrentDrawAmps();
        inputs.rightCurrentAmps = rightSim.getCurrentDrawAmps();
    }
    
    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftAppliedVolts = leftVolts;
        rightAppliedVolts = rightVolts;
        leftSim.setInputVoltage(leftVolts);
        rightSim.setInputVoltage(rightVolts);
    }
}
```

### Subsystem Integration
```java
public class DriveSubsystem extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    
    public DriveSubsystem(DriveIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
    }
    
    public void setVoltage(double leftVolts, double rightVolts) {
        io.setVoltage(leftVolts, rightVolts);
    }
}
```

## Logging Features

### Basic Logging
```java
import org.littletonrobotics.junction.Logger;

// Log simple values
Logger.recordOutput("Drive/LeftVelocity", leftVelocityRPM);
Logger.recordOutput("Drive/RightVelocity", rightVelocityRPM);

// Log arrays
Logger.recordOutput("Drive/ModuleStates", moduleStates);

// Log poses and trajectories
Logger.recordOutput("Odometry/Robot", robotPose);
Logger.recordOutput("Trajectory/Target", targetTrajectory);
```

### Auto-logging with @AutoLog
The `@AutoLog` annotation automatically generates logging code for data classes:

```java
public class ExampleIOInputs {
    @AutoLog
    public static class ExampleIOInputs {
        public double velocity = 0.0;
        public double position = 0.0;
        public boolean isEnabled = false;
        public double[] voltages = new double[] {};
    }
}
```

## Configuration and Setup

### Robot Main Class
```java
public class Main {
    public static void main(String... args) {
        // Configure logging before robot initialization
        Logger.recordMetadata("ProjectName", "MyRobot");
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("GitDirty", BuildConstants.DIRTY);
        
        // Set recording directory
        if (Robot.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.setReplaySource(new WPILOGReader("path/to/log.wpilog"));
            Logger.addDataReceiver(new WPILOGWriter(""));
        }
        
        Logger.start();
        
        RobotBase.startRobot(Robot::new);
    }
}
```

### Build Configuration
The build.gradle already includes AdvantageKit configuration:

```gradle
def akitJson = new groovy.json.JsonSlurper().parseText(new File(projectDir.getAbsolutePath() + "/vendordeps/AdvantageKit.json").text)
annotationProcessor "org.littletonrobotics.akit:akit-autolog:$akitJson.version"
```

## Data Analysis

### AdvantageScope
AdvantageScope is the companion application for viewing and analyzing logged data:

- **Installation**: Download from GitHub releases
- **Connection**: Connect to robot via NetworkTables or load saved logs
- **Features**:
  - Real-time data plotting
  - 3D field visualization
  - Odometry replay
  - Video synchronization
  - Custom dashboard creation

### Log File Management
- **Format**: WPILOG binary format
- **Location**: `/home/lvuser/logs` on robot
- **Retrieval**: Use FTP, web interface, or USB
- **Replay**: Load logs for simulation and analysis

## Performance Optimization

### Status Frame Configuration
```java
// Optimize CAN frame rates for logging
public void configureStatusFrames() {
    // Reduce non-critical frame rates
    motor.getPosition().setUpdateFrequency(10); // 10 Hz instead of 50 Hz
    motor.getStatorCurrent().setUpdateFrequency(20); // 20 Hz
    
    // Keep important signals at high frequency
    motor.getVelocity().setUpdateFrequency(100); // 100 Hz
}
```

### Selective Logging
```java
// Log conditionally to reduce data volume
if (DriverStation.isEnabled() && DriverStation.isAutonomous()) {
    Logger.recordOutput("Auto/DetailedData", detailedArray);
}
```

## Integration Patterns Used in This Codebase

Based on the import analysis, this robot uses AdvantageKit for:

- **Robot Main Class**: Extends LoggedRobot for automatic logging
- **All Subsystems**: Use IO layer pattern with AutoLogged inputs
- **Simulation**: Comprehensive physics simulation with logged data
- **Vision System**: Log camera poses and target information
- **Autonomous**: Log trajectory following and command execution

## Best Practices

1. **IO Layer Separation**: Always separate hardware interfaces from business logic
2. **Comprehensive Logging**: Log all sensor inputs, outputs, and intermediate calculations
3. **Simulation Parity**: Ensure simulation matches real hardware behavior
4. **Metadata Recording**: Include build information and Git status
5. **Performance Monitoring**: Monitor log file sizes and network bandwidth
6. **Regular Analysis**: Review logs after matches and practice sessions

## Troubleshooting

1. **Large Log Files**: Adjust logging frequency and selective logging
2. **Network Bandwidth**: Use NT4Publisher sparingly during competition
3. **Replay Issues**: Ensure all IO implementations handle replay correctly
4. **Build Errors**: Check annotation processor configuration in build.gradle

## Additional Resources
- [AdvantageKit Documentation](https://github.com/Mechanical-Advantage/AdvantageKit)
- [AdvantageScope Releases](https://github.com/Mechanical-Advantage/AdvantageScope/releases)
- [6328 Programming Resources](https://www.team6328.org/programming)