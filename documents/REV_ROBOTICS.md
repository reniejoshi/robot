# REV Robotics Integration Guide

## Overview

REV Robotics provides comprehensive motor control solutions for FRC teams, including brushless NEO motors, SPARK MAX motor controllers, and the REVLib programming library. This guide covers integration patterns used in the Bear Metal robot codebase.

## Core Components

### SPARK MAX Motor Controller
- Advanced brushed and brushless DC motor controller
- PWM, CAN, and USB connectivity
- Automatic signal type detection
- Integrated with REV NEO motors for optimal performance
- Standard CAN frames with extended ID (29 bits)

### NEO Brushless Motor
- First brushless motor designed specifically for FRC
- Integrated encoder for precise feedback
- Optimized for SPARK MAX controller
- High power-to-weight ratio

### REVLib Java Library
- Official Java API for REV Robotics hardware
- Comprehensive motor control and configuration
- Encoder and sensor integration
- PID control and motion profiling

## Installation and Setup

### Dependency Management
REVLib is typically included via WPILib's vendordeps system:

```json
{
    "fileName": "REVLib.json",
    "name": "REVLib",
    "version": "2024.2.4",
    "mavenUrls": [
        "https://maven.revrobotics.com/"
    ],
    "jsonUrl": "https://software-metadata.revrobotics.com/REVLib-frc2024.json"
}
```

### Basic Import Statements
```java
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
```

## Motor Configuration Patterns

### Basic SPARK MAX Initialization
```java
public class ExampleConstants {
    public static final int MOTOR_CAN_ID = 10;
    
    public static SparkMaxConfig createMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Motor output settings
        config.inverted(false);
        config.idleMode(SparkMaxConfig.IdleMode.kBrake);
        
        // Current limiting
        config.smartCurrentLimit(40); // Amps
        
        // Encoder settings
        config.encoder.positionConversionFactor(1.0); // Rotations
        config.encoder.velocityConversionFactor(1.0); // RPM
        
        return config;
    }
}

public class ExampleSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    
    public ExampleSubsystem() {
        motor = new SparkMax(ExampleConstants.MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
        
        // Configure motor with error handling
        SparkMaxConfig config = ExampleConstants.createMotorConfig();
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, 
                       SparkBase.PersistMode.kPersistParameters);
        
        encoder = motor.getEncoder();
    }
}
```

### Follower Motor Configuration
```java
public class DualMotorSubsystem extends SubsystemBase {
    private final SparkMax leader;
    private final SparkMax follower;
    
    public DualMotorSubsystem() {
        leader = new SparkMax(LEADER_CAN_ID, SparkLowLevel.MotorType.kBrushless);
        follower = new SparkMax(FOLLOWER_CAN_ID, SparkLowLevel.MotorType.kBrushless);
        
        // Configure leader
        SparkMaxConfig leaderConfig = createMotorConfig();
        leader.configure(leaderConfig, SparkBase.ResetMode.kResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters);
        
        // Configure follower to follow leader
        SparkMaxConfig followerConfig = createMotorConfig();
        followerConfig.follow(leader.getDeviceId(), true); // true = inverted
        follower.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters,
                          SparkBase.PersistMode.kPersistParameters);
    }
}
```

## Advanced Motor Control

### PID Control Configuration
```java
public static SparkMaxConfig createPIDConfig() {
    SparkMaxConfig config = new SparkMaxConfig();
    
    // PID coefficients for slot 0
    config.closedLoop.pid(0.0001, 0.000001, 0.0, 0.0); // kP, kI, kD, kFF
    config.closedLoop.outputRange(-1.0, 1.0);
    
    // Soft limits
    config.softLimit.forwardSoftLimit(100.0); // Rotations
    config.softLimit.forwardSoftLimitEnabled(true);
    config.softLimit.reverseSoftLimit(0.0);
    config.softLimit.reverseSoftLimitEnabled(true);
    
    return config;
}

// Position control usage
public void setPosition(double positionRotations) {
    motor.getClosedLoopController().setReference(
        positionRotations, 
        SparkMaxClosedLoopController.ControlType.kPosition
    );
}

// Velocity control usage  
public void setVelocity(double velocityRPM) {
    motor.getClosedLoopController().setReference(
        velocityRPM,
        SparkMaxClosedLoopController.ControlType.kVelocity
    );
}
```

### Smart Motion (Trapezoidal Profiles)
```java
public static SparkMaxConfig createSmartMotionConfig() {
    SparkMaxConfig config = createPIDConfig();
    
    // Smart Motion parameters
    config.closedLoop.smartMotion.maxVelocity(2000.0); // RPM
    config.closedLoop.smartMotion.maxAcceleration(1500.0); // RPM/sec
    config.closedLoop.smartMotion.allowedClosedLoopError(1.0); // Rotations
    
    return config;
}

public void setSmartMotionPosition(double targetPosition) {
    motor.getClosedLoopController().setReference(
        targetPosition,
        SparkMaxClosedLoopController.ControlType.kSmartMotion
    );
}
```

## Sensor Integration

### Built-in Encoder Usage
```java
public class EncoderSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    
    public EncoderSubsystem() {
        motor = new SparkMax(MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        
        // Configure encoder conversion factors
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(GEAR_RATIO); // Convert to mechanism rotations
        config.encoder.velocityConversionFactor(GEAR_RATIO); // Convert to mechanism RPM
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters,
                       SparkBase.PersistMode.kPersistParameters);
    }
    
    public double getPosition() {
        return encoder.getPosition(); // In mechanism rotations
    }
    
    public double getVelocity() {
        return encoder.getVelocity(); // In mechanism RPM
    }
    
    public void resetEncoder() {
        encoder.setPosition(0.0);
    }
}
```

### Limit Switch Integration
```java
public class LimitSwitchSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final SparkLimitSwitch forwardLimit;
    private final SparkLimitSwitch reverseLimit;
    
    public LimitSwitchSubsystem() {
        motor = new SparkMax(MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
        
        // Get limit switches
        forwardLimit = motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        reverseLimit = motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        
        // Enable limit switches
        SparkMaxConfig config = new SparkMaxConfig();
        config.limitSwitch.forwardLimitSwitchEnabled(true);
        config.limitSwitch.reverseLimitSwitchEnabled(true);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters,
                       SparkBase.PersistMode.kPersistParameters);
    }
    
    public boolean isForwardLimitPressed() {
        return forwardLimit.isPressed();
    }
    
    public boolean isReverseLimitPressed() {
        return reverseLimit.isPressed();
    }
}
```

## Error Handling and Diagnostics

### Fault Monitoring
```java
public void checkMotorHealth() {
    // Check for faults
    int faults = motor.getFaults();
    int stickyFaults = motor.getStickyFaults();
    
    // Log fault conditions
    if (faults != 0) {
        Logger.recordOutput("Motor/Faults", faults);
    }
    
    // Clear sticky faults if needed
    if (stickyFaults != 0) {
        motor.clearFaults();
    }
    
    // Monitor temperature
    double temperature = motor.getMotorTemperature();
    Logger.recordOutput("Motor/Temperature", temperature);
    
    // Monitor applied output and current
    Logger.recordOutput("Motor/AppliedOutput", motor.getAppliedOutput());
    Logger.recordOutput("Motor/OutputCurrent", motor.getOutputCurrent());
}
```

### Configuration Validation
```java
public boolean validateConfiguration() {
    // Check if configuration was applied successfully
    // REV motors will report configuration errors via faults
    
    int faults = motor.getFaults();
    boolean configValid = (faults & SparkMaxFault.kHardLimitFwd.value) == 0 &&
                         (faults & SparkMaxFault.kHardLimitRev.value) == 0;
    
    return configValid;
}
```

## Simulation Support

### SPARK MAX Simulation
```java
public class SimulatedSparkMax extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    
    // Simulation state
    private double simulatedPosition = 0.0;
    private double simulatedVelocity = 0.0;
    
    public SimulatedSparkMax() {
        motor = new SparkMax(MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
    }
    
    @Override
    public void simulationPeriodic() {
        // Simple simulation model
        double motorVoltage = motor.getAppliedOutput() * 12.0; // Convert to volts
        
        // Simple velocity calculation (replace with proper physics model)
        simulatedVelocity = motorVoltage * MAX_RPM / 12.0;
        
        // Update position based on velocity
        simulatedPosition += simulatedVelocity * 0.02 / 60.0; // 20ms period, convert RPM to rotations
        
        // Update encoder simulation
        motor.getEncoder().setPosition(simulatedPosition);
    }
}
```

## Common Patterns in Bear Metal Codebase

### Constants Structure
```java
public class SubsystemConstants {
    // Hardware IDs
    public static final int MOTOR_CAN_ID = 10;
    
    // Physical constants
    public static final double GEAR_RATIO = 10.5;
    public static final double WHEEL_DIAMETER_METERS = 0.1524; // 6 inches
    
    // Control constants
    public static final double MAX_VELOCITY_RPM = 5400.0;
    public static final double MAX_CURRENT_AMPS = 40.0;
    
    // PID gains
    public static final double KP = 0.0001;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KFF = 0.0;
    
    public static SparkMaxConfig createConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.inverted(false);
        config.idleMode(SparkMaxConfig.IdleMode.kBrake);
        config.smartCurrentLimit((int) MAX_CURRENT_AMPS);
        
        // PID configuration
        config.closedLoop.pid(KP, KI, KD, KFF);
        config.closedLoop.outputRange(-1.0, 1.0);
        
        // Encoder conversion factors
        config.encoder.positionConversionFactor(1.0 / GEAR_RATIO); // Mechanism rotations
        config.encoder.velocityConversionFactor(1.0 / GEAR_RATIO); // Mechanism RPM
        
        return config;
    }
}
```

### Integration with AbstractSubsystem
```java
public class ExampleSubsystem extends AbstractSubsystem {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    
    public ExampleSubsystem() {
        motor = new SparkMax(ExampleConstants.MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        
        // Configure motor
        SparkMaxConfig config = ExampleConstants.createConfig();
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters,
                       SparkBase.PersistMode.kPersistParameters);
    }
    
    @Override
    public void subsystemPeriodic() {
        // Log telemetry with AdvantageKit
        Logger.recordOutput("Example/Position", encoder.getPosition());
        Logger.recordOutput("Example/Velocity", encoder.getVelocity());
        Logger.recordOutput("Example/Current", motor.getOutputCurrent());
        Logger.recordOutput("Example/Temperature", motor.getMotorTemperature());
        Logger.recordOutput("Example/AppliedOutput", motor.getAppliedOutput());
    }
    
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
    
    public void setPosition(double position) {
        motor.getClosedLoopController().setReference(
            position, 
            SparkMaxClosedLoopController.ControlType.kPosition
        );
    }
    
    public void stop() {
        motor.stopMotor();
    }
    
    @Override
    public void close() {
        motor.close();
    }
}
```

## Best Practices

### Configuration Management
1. **Always use SparkMaxConfig objects** for centralized configuration
2. **Apply ResetMode.kResetSafeParameters** to ensure clean state
3. **Use PersistMode.kPersistParameters** to save settings across power cycles
4. **Validate configuration** by monitoring faults after setup

### Current Limiting
1. **Set appropriate smart current limits** (typically 40A for mechanisms, 60A for drivetrain)
2. **Monitor output current** in telemetry for debugging
3. **Use supply current limiting** to protect electrical system

### Encoder Best Practices
1. **Set conversion factors** to get mechanism units directly
2. **Reset encoder position** at known mechanical positions
3. **Monitor encoder health** through velocity and position consistency

### Error Handling
1. **Monitor motor faults** in periodic functions
2. **Clear sticky faults** when appropriate
3. **Log diagnostic information** for debugging
4. **Implement fallback behaviors** for fault conditions

### Performance Optimization
1. **Use follower configuration** instead of duplicate control signals
2. **Optimize CAN bus usage** with appropriate update rates
3. **Minimize object creation** in periodic functions
4. **Cache frequently accessed values**

## Hardware Setup Considerations

### CAN Bus Configuration
- Use consistent CAN ID assignment across team
- Avoid ID conflicts with other CAN devices
- Consider CAN bus loading with multiple SPARK MAX controllers

### Wiring Best Practices
- Secure motor connections to prevent intermittent faults
- Use proper wire gauge for current requirements
- Implement proper grounding to reduce electrical noise

### Mechanical Integration
- Ensure encoder alignment for accurate feedback
- Consider gear backlash in position control applications
- Implement proper mechanical stops for safety

This guide provides the foundation for integrating REV Robotics hardware into FRC robot code using the patterns established in the Bear Metal codebase.