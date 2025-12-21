# TinyLog API Documentation

## Overview
TinyLog is a lightweight, high-performance logging framework for Java. It provides a simple, clean API for logging with minimal overhead and excellent performance characteristics, making it ideal for FRC robot applications where performance is critical.

## Version Information
- **Library Version**: 2.7.0
- **Group ID**: org.tinylog
- **Artifacts**: 
  - tinylog-api
  - tinylog-impl
- **Official Website**: https://tinylog.org/

## Configuration
TinyLog is configured via the `tinylog.properties` file located in `src/main/resources/`:

```properties
# Current configuration in this project
writer=console
writer.level=info
writer.format={date: HH:mm:ss.SSS} {level} {class-name} - {message}
writingthread=true
```

### Configuration Options

#### Writers
```properties
# Console output
writer=console

# File output
writer=file
writer.file=robot.log

# Rolling file (size-based)
writer=rolling file
writer.file=logs/robot-{count}.log
writer.policies=size: 10MB
writer.backups=5

# Multiple writers
writer1=console
writer2=file
writer2.file=robot.log
```

#### Log Levels
```properties
# Global level
level=info

# Package-specific levels
level@org.tahomarobotics=debug
level@com.ctre=warn
```

#### Format Patterns
```properties
# Timestamp formats
writer.format={date: HH:mm:ss.SSS} {level} {class-name} - {message}
writer.format={date: yyyy-MM-dd HH:mm:ss} [{level}] {class}: {message}

# Include thread information
writer.format={date} {level} [{thread}] {class-name}: {message}

# Include method and line number
writer.format={date} {level} {class-name}.{method}({line}): {message}
```

## Basic Usage

### Static Logger Import
```java
import org.tinylog.Logger;

public class DriveSubsystem {
    public void initialize() {
        Logger.info("Initializing drive subsystem");
        Logger.debug("Motor configuration: {}", motorConfig);
    }
    
    public void reportError(Exception e) {
        Logger.error(e, "Drive system error occurred");
    }
}
```

### Log Levels
```java
// Different log levels
Logger.trace("Very detailed debug information");
Logger.debug("Debug information for development");
Logger.info("General information about program execution");
Logger.warn("Warning about potential issues");
Logger.error("Error conditions that should be investigated");
```

### Parameterized Messages
```java
// Efficient parameterized logging (no string concatenation unless logged)
Logger.info("Robot position: x={}, y={}, theta={}", 
    pose.getX(), pose.getY(), pose.getRotation().getDegrees());

Logger.debug("Motor {} velocity: {} RPM, current: {} A", 
    motorId, velocity, current);

// Exception logging
try {
    // risky operation
} catch (Exception e) {
    Logger.error(e, "Failed to execute command: {}", commandName);
}
```

## Usage Patterns in This Codebase

Based on the import analysis, TinyLog is used throughout the robot code:

### Utility Classes
- **RobustConfigurator**: Logging configuration attempts and failures
- **WatchDog**: Timing and performance monitoring
- **CommandLogger**: Command execution tracking

### Subsystem Classes
- **Chassis**: Drive system status and diagnostics  
- **Windmill**: Arm/elevator positioning and errors
- **VisionSimulation**: Computer vision processing logs

### Typical Usage Examples

#### Motor Configuration Logging
```java
public class ChassisConstants {
    public void configureMotor(TalonFX motor, String name) {
        Logger.info("Configuring motor: {}", name);
        
        try {
            motor.getConfigurator().apply(config);
            Logger.debug("Motor {} configured successfully", name);
        } catch (Exception e) {
            Logger.error(e, "Failed to configure motor: {}", name);
        }
    }
}
```

#### Watchdog Monitoring
```java
public class WatchDog {
    public void checkPerformance() {
        double loopTime = getLoopTime();
        if (loopTime > MAX_LOOP_TIME) {
            Logger.warn("Slow loop detected: {:.2f}ms (max: {:.2f}ms)", 
                loopTime * 1000, MAX_LOOP_TIME * 1000);
        }
        
        Logger.trace("Loop time: {:.3f}ms", loopTime * 1000);
    }
}
```

#### Command Execution Tracking
```java
public class CommandLogger {
    public void logCommandStart(Command command) {
        Logger.info("Starting command: {}", command.getName());
    }
    
    public void logCommandEnd(Command command, boolean interrupted) {
        if (interrupted) {
            Logger.warn("Command interrupted: {}", command.getName());
        } else {
            Logger.info("Command completed: {}", command.getName());
        }
    }
}
```

## Performance Considerations

### Async Logging
```properties
# Enable writing thread for better performance
writingthread=true

# Configure thread buffer size
writingthread.buffer=8192
```

### Conditional Logging
```java
// Expensive operations only executed when level is enabled
if (Logger.isDebugEnabled()) {
    Logger.debug("Complex debug info: {}", createComplexDebugString());
}

// Or use lambda for lazy evaluation (TinyLog 2.0+)
Logger.debug(() -> "Complex debug info: " + createComplexDebugString());
```

### Level Guards
```java
public class HighFrequencySubsystem {
    private static final boolean DEBUG_ENABLED = Logger.isDebugEnabled();
    
    @Override
    public void periodic() {
        // Only log in debug builds to avoid performance impact
        if (DEBUG_ENABLED) {
            Logger.debug("Sensor readings: gyro={}, encoder={}", 
                gyroAngle, encoderPosition);
        }
    }
}
```

## Robot-Specific Logging Patterns

### Autonomous Logging
```java
public class AutonomousCommand {
    @Override
    public void initialize() {
        Logger.info("Starting autonomous: {}", getName());
    }
    
    @Override
    public void execute() {
        Logger.debug("Auto step: {}, elapsed: {:.1f}s", 
            getCurrentStep(), timer.get());
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Logger.warn("Autonomous interrupted at step: {}", getCurrentStep());
        } else {
            Logger.info("Autonomous completed successfully");
        }
    }
}
```

### Subsystem Health Monitoring
```java
public class DriveSubsystem {
    @Override
    public void periodic() {
        // Log critical errors
        if (hasMotorFault()) {
            Logger.error("Drive motor fault detected: {}", getFaultDescription());
        }
        
        // Log warnings for degraded performance
        if (getBatteryVoltage() < MIN_VOLTAGE) {
            Logger.warn("Low battery voltage: {:.1f}V", getBatteryVoltage());
        }
        
        // Debug-level telemetry
        Logger.debug("Drive odometry: {}", getPose());
    }
}
```

## Integration with Robot Framework

### Robot Main Class
```java
public class Robot extends LoggedRobot {
    @Override
    public void robotInit() {
        Logger.info("Robot initializing - Team 2046 Bear Metal");
        Logger.info("Build info - Date: {}, Git: {}", 
            BuildConstants.BUILD_DATE, BuildConstants.GIT_SHA);
    }
    
    @Override
    public void disabledInit() {
        Logger.info("Robot disabled");
    }
    
    @Override
    public void autonomousInit() {
        Logger.info("Autonomous mode started");
    }
    
    @Override
    public void teleopInit() {
        Logger.info("Teleop mode started");
    }
}
```

### Competition Logging
```java
// Different log levels for competition vs practice
public class RobotContainer {
    public RobotContainer() {
        if (DriverStation.isFMSAttached()) {
            Logger.info("FMS detected - competition mode logging");
            // Reduce logging verbosity for competition
        } else {
            Logger.info("Practice mode - full logging enabled");
        }
    }
}
```

## Advanced Configuration

### Environment-Specific Configuration
```properties
# Development configuration
writer=console
writer.level=debug
writer.format={date: HH:mm:ss.SSS} {level} {class-name}.{method}({line}) - {message}

# Competition configuration  
writer=file
writer.file=/home/lvuser/logs/robot.log
writer.level=info
writer.format={date: yyyy-MM-dd HH:mm:ss} {level} {class-name} - {message}
writingthread=true
```

### Custom Log Formatting
```java
// Custom format for specific use cases
public class TelemetryLogger {
    public static void logSensorData(String sensor, double value, String unit) {
        Logger.info("SENSOR,{},{},{}", sensor, value, unit);
    }
    
    public static void logMatchEvent(String event) {
        Logger.info("MATCH_EVENT,{},{}", event, Timer.getFPGATimestamp());
    }
}
```

## Best Practices for FRC

1. **Use Appropriate Log Levels**: Info for important events, debug for detailed diagnostics, warn/error for problems
2. **Parameterized Messages**: Use `{}` placeholders instead of string concatenation for performance
3. **Exception Logging**: Always include the exception when logging errors
4. **Competition Readiness**: Reduce log verbosity for competition to improve performance
5. **Structured Logging**: Use consistent formats for easier log analysis
6. **Performance Monitoring**: Use trace level for high-frequency debugging that can be easily disabled

## Troubleshooting

1. **No Log Output**: Check tinylog.properties is in resources folder and level settings
2. **Performance Issues**: Enable writingthread and reduce log level for competition
3. **File Not Created**: Verify file path permissions and disk space
4. **Missing Messages**: Check if log level filters are excluding desired messages

## Additional Resources
- [TinyLog Official Documentation](https://tinylog.org/v2/)
- [Configuration Reference](https://tinylog.org/v2/configuration/)
- [Performance Benchmarks](https://tinylog.org/v2/benchmark/)