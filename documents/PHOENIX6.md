# Phoenix 6 (CTRE) API Documentation

## Overview
Phoenix 6 is CTRE's latest API for controlling motor controllers, sensors, and other CAN devices. This version 25.2.1 provides comprehensive motor control capabilities for TalonFX, CANCoder, and Pigeon2 devices.

## Version Information
- **Library Version**: 25.2.1
- **Java Group ID**: com.ctre.phoenix6
- **Artifact**: wpiapi-java

## Key Components

### CANivore CAN Bus Controller

The CANivore is CTRE's dedicated CAN bus controller that provides isolated, high-performance CAN communication and enables Phoenix Pro features.

#### CANivore Benefits
- **Isolated CAN Bus**: Separate from roboRIO's CAN bus for better reliability
- **Higher Bandwidth**: Reduced bus utilization with faster communication
- **Phoenix Pro Features**: Required for advanced Pro-licensed features
- **Time Synchronization**: Precise timing for coordinated device control
- **Better Diagnostics**: Enhanced CAN bus monitoring and fault detection

#### CANivore Setup
```java
// Initialize devices on CANivore bus
// The second parameter specifies the CANivore name (default is "canivore")
TalonFX leftMotor = new TalonFX(1, "canivore");
TalonFX rightMotor = new TalonFX(2, "canivore"); 
CANCoder encoder = new CANCoder(10, "canivore");
Pigeon2 gyro = new Pigeon2(15, "canivore");

// Alternative: Use RobotMap constants for bus names
public static final String CANIVORE_NAME = "canivore";
TalonFX motor = new TalonFX(1, RobotMap.CANIVORE_NAME);
```

#### CANivore Configuration
```java
// Check CANivore connection status
var canivoreStatus = CANBus.getStatus("canivore");
Logger.recordOutput("CANivore/BusUtilization", canivoreStatus.BusUtilization);
Logger.recordOutput("CANivore/BusOffCount", canivoreStatus.BusOffCount);
Logger.recordOutput("CANivore/TxFullCount", canivoreStatus.TxFullCount);
Logger.recordOutput("CANivore/ReceiveErrorCount", canivoreStatus.RxErrorCount);
Logger.recordOutput("CANivore/TransmitErrorCount", canivoreStatus.TxErrorCount);
```

#### Mixed CAN Bus Usage
```java
// Some devices on roboRIO CAN, others on CANivore
TalonFX driveMotor = new TalonFX(1, "canivore");     // High-performance drivetrain
TalonFX armMotor = new TalonFX(10, "rio");           // Arm on roboRIO CAN
TalonFX intakeMotor = new TalonFX(11, "rio");        // Intake on roboRIO CAN

// Note: Device IDs can overlap between buses
TalonFX leftDrive = new TalonFX(1, "canivore");      // ID 1 on CANivore
TalonFX leftArm = new TalonFX(1, "rio");             // ID 1 on roboRIO (different bus)
```

### Phoenix Pro Licensing

Phoenix Pro provides advanced features that require licensing. Pro licenses are tied to specific devices and enable enhanced functionality.

#### Pro Features Overview
- **Advanced Motion Control**: Enhanced Motion Magic with S-curves
- **Synchronized Control**: Time-synchronized multi-device control  
- **Signal Logging**: High-frequency data logging
- **Tuning Tools**: Professional tuning and analysis tools
- **Advanced Diagnostics**: Detailed device health monitoring
- **Custom Control Loops**: User-defined control algorithms

#### Pro License Management
```java
// Check if device has Pro license
var licenseStatus = motor.hasProLicense();
Logger.recordOutput("Motor/HasProLicense", licenseStatus);

// Pro features will automatically be available if license is present
// No code changes needed - APIs detect license and enable features
```

#### Pro-Only Motion Magic Features
```java
// Motion Magic Pro with S-curve profiling (requires Pro license)
var talonFXConfigs = new TalonFXConfiguration();

// Enhanced Motion Magic with S-curves
var motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicCruiseVelocity = 80;      // rps
motionMagicConfigs.MotionMagicAcceleration = 160;       // rps/s  
motionMagicConfigs.MotionMagicJerk = 1600;              // rps/s² (Pro only)
motionMagicConfigs.MotionMagicExpo_kV = 0.12;           // Pro velocity feedforward
motionMagicConfigs.MotionMagicExpo_kA = 0.01;           // Pro acceleration feedforward

motor.getConfigurator().apply(talonFXConfigs);

// Use enhanced Motion Magic
final MotionMagicExpoVoltage m_proRequest = new MotionMagicExpoVoltage(0);
motor.setControl(m_proRequest.withPosition(100));
```

#### Synchronized Control (Pro Feature)
```java
// Synchronize multiple devices with precise timing
// This requires CANivore and Pro licenses on all devices
var orchestrator = new Orchestrator();

// Add devices to synchronized group
orchestrator.addDevice(leftMotor);
orchestrator.addDevice(rightMotor); 
orchestrator.addDevice(armMotor);

// Execute synchronized control
orchestrator.executeAll(() -> {
    leftMotor.setControl(leftRequest.withPosition(leftTarget));
    rightMotor.setControl(rightRequest.withPosition(rightTarget));
    armMotor.setControl(armRequest.withPosition(armTarget));
});
```

#### Pro Signal Logging
```java
// High-frequency signal logging (Pro feature)
var positionSignal = motor.getPosition();
var velocitySignal = motor.getVelocity();
var currentSignal = motor.getStatorCurrent();

// Configure high-frequency logging (Pro only)
BaseStatusSignal.setUpdateFrequencyForAll(250.0,  // 250Hz update rate
    positionSignal, velocitySignal, currentSignal);

// Log Pro signals
public void logProSignals() {
    if (motor.hasProLicense()) {
        Logger.recordOutput("Motor/Position_Pro", positionSignal.getValue());
        Logger.recordOutput("Motor/Velocity_Pro", velocitySignal.getValue());  
        Logger.recordOutput("Motor/Current_Pro", currentSignal.getValue());
        
        // Additional Pro diagnostic signals
        Logger.recordOutput("Motor/Temperature", motor.getDeviceTemp().getValue());
        Logger.recordOutput("Motor/SupplyVoltage", motor.getSupplyVoltage().getValue());
    }
}
```

#### Pro vs Standard Feature Comparison

| Feature | Standard | Pro |
|---------|----------|-----|
| Basic Control | ✅ | ✅ |
| Motion Magic | ✅ | ✅ Enhanced |
| S-Curve Profiling | ❌ | ✅ |
| Synchronized Control | ❌ | ✅ |
| High-Freq Logging | ❌ | ✅ |
| Advanced Tuning | ❌ | ✅ |
| Signal Frequency | 50Hz max | 250Hz+ |
| Jerk Control | ❌ | ✅ |
| Time Sync | ❌ | ✅ |

### TalonFX Motor Controller
The TalonFX is CTRE's flagship motor controller, supporting various control modes and advanced features.

#### Basic Configuration
```java
// Initialize TalonFX with device ID and CAN bus
TalonFX m_motor = new TalonFX(0, "canivore");

// Configure motor output inversion
var motorConfig = new MotorOutputConfigs();
motorConfig.Inverted = InvertedValue.CounterClockwise_Positive;
m_motor.getConfigurator().apply(motorConfig);
```

#### PID Control Configuration
```java
// Configure Slot 0 PID gains
var slot0Configs = new Slot0Configs();
slot0Configs.kS = 0.25; // Static friction compensation (Volts)
slot0Configs.kV = 0.12; // Velocity gain (V per rps)
slot0Configs.kA = 0.01; // Acceleration gain (V per rps/s)
slot0Configs.kP = 4.8;  // Proportional gain
slot0Configs.kI = 0;    // Integral gain
slot0Configs.kD = 0.1;  // Derivative gain

m_motor.getConfigurator().apply(slot0Configs);
```

#### Motion Magic Configuration
```java
// Motion Magic provides smooth trapezoidal motion profiling
var talonFXConfigs = new TalonFXConfiguration();

// Set Motion Magic parameters
var motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicCruiseVelocity = 80;  // 80 rps cruise velocity
motionMagicConfigs.MotionMagicAcceleration = 160;   // 160 rps/s acceleration
motionMagicConfigs.MotionMagicJerk = 1600;          // 1600 rps/s² jerk

m_motor.getConfigurator().apply(talonFXConfigs);

// Use Motion Magic in periodic
final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
m_motor.setControl(m_request.withPosition(100)); // Target 100 rotations
```

### Control Requests
Phoenix 6 uses a control request system for commanding motors:

#### Open-Loop Control
```java
// Duty cycle control (-1.0 to 1.0)
final DutyCycleOut m_dutyCycle = new DutyCycleOut(0.0);
m_motor.setControl(m_dutyCycle.withOutput(0.5)); // 50% output
```

#### Closed-Loop Control
```java
// Position control
final PositionVoltage m_posRequest = new PositionVoltage(0).withSlot(0);
m_motor.setControl(m_posRequest.withPosition(10)); // 10 rotations

// Velocity control
final VelocityVoltage m_velRequest = new VelocityVoltage(0).withSlot(0);
m_motor.setControl(m_velRequest.withVelocity(50)); // 50 rps
```

### Follower Configuration
```java
// Configure follower motors
m_follower.setControl(new Follower(m_leader.getDeviceID(), false));
m_strictFollower.setControl(new StrictFollower(m_leader.getDeviceID()));
```

### Current Limiting
```java
// Configure stator current limits
var limitConfigs = new CurrentLimitsConfigs();
limitConfigs.StatorCurrentLimit = 120;        // 120 A limit
limitConfigs.StatorCurrentLimitEnable = true;
m_motor.getConfigurator().apply(limitConfigs);
```

### Remote Sensors
```java
// Use CANCoder as feedback sensor
var fx_cfg = new TalonFXConfiguration();
fx_cfg.Feedback.FeedbackRemoteSensorID = m_cancoder.getDeviceID();
fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
m_motor.getConfigurator().apply(fx_cfg);
```

## Simulation Support
Phoenix 6 provides comprehensive simulation support:

```java
// Simulation integration with WPILib DCMotorSim
public void simulationPeriodic() {
    var talonFXSim = m_motor.getSimState();
    
    // Set supply voltage
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    
    // Get motor voltage for physics simulation
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();
    
    // Update physics model
    m_motorSim.setInputVoltage(motorVoltage.in(Volts));
    m_motorSim.update(0.020);
    
    // Apply results back to simulation
    talonFXSim.setRawRotorPosition(m_motorSim.getAngularPosition().times(kGearRatio));
    talonFXSim.setRotorVelocity(m_motorSim.getAngularVelocity().times(kGearRatio));
}
```

## Integration with WPILib
```java
// Use with DifferentialDrive
final DifferentialDrive m_diffDrive = new DifferentialDrive(
    m_leftMotor::set, m_rightMotor::set
);
```

## Best Practices

1. **Reuse Control Requests**: Create control request objects once and reuse them for better performance.

2. **Configuration Management**: Use `TalonFXConfiguration` objects to manage multiple configuration parameters.

3. **Timeout Handling**: Apply configurations with appropriate timeouts:
   ```java
   m_motor.getConfigurator().apply(configs, 0.050); // 50ms timeout
   ```

4. **Status Frame Optimization**: Adjust status frame rates for performance optimization.

5. **Unit Testing**: Phoenix 6 provides simulation support for comprehensive unit testing.

6. **CANivore Usage Strategy**: 
   - Use CANivore for high-performance subsystems (drivetrain, critical mechanisms)
   - Keep lower-priority devices on roboRIO CAN to balance cost and performance
   - Monitor CAN bus utilization with `CANBus.getStatus()`

7. **Pro License Management**:
   - Check license status during initialization: `motor.hasProLicense()`
   - Implement fallback behaviors for devices without Pro licenses
   - Use Pro features conditionally to maintain compatibility
   ```java
   if (motor.hasProLicense()) {
       // Use advanced Pro features
       BaseStatusSignal.setUpdateFrequencyForAll(250.0, signals);
   } else {
       // Fall back to standard features  
       BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals);
   }
   ```

8. **CAN Bus Architecture**:
   - Assign device IDs systematically across buses
   - Document which devices are on which bus
   - Consider CAN bus loading when adding new devices
   ```java
   // Example ID assignment strategy
   // CANivore bus: 1-10 (drivetrain), 11-20 (arm/elevator)
   // roboRIO bus: 21-30 (intake), 31-40 (accessories)
   ```

9. **Signal Update Frequency Optimization**:
   - Use higher frequencies (100-250Hz) only for critical control loops
   - Use standard frequencies (50Hz) for telemetry and non-critical data
   - Group signals by update frequency for efficiency
   ```java
   // Critical control signals - high frequency
   BaseStatusSignal.setUpdateFrequencyForAll(100.0, 
       motor.getPosition(), motor.getVelocity());
   
   // Diagnostic signals - standard frequency
   BaseStatusSignal.setUpdateFrequencyForAll(10.0,
       motor.getDeviceTemp(), motor.getSupplyVoltage());
   ```

## Common Patterns Used in This Codebase

Based on the import analysis, this robot uses Phoenix 6 for:
- **Chassis**: Swerve drive motors and steering encoders
- **Windmill**: Arm and elevator motor control
- **Collector**: Intake mechanism motors
- **Grabber**: Game piece manipulation motors
- **Simulation**: Physics-based motor simulation

## Troubleshooting

1. **CAN Bus Configuration**: Ensure correct CAN bus name ("canivore" vs "rio")
2. **Device IDs**: Verify unique device IDs for all Phoenix devices
3. **Firmware Updates**: Keep firmware up to date for latest features
4. **Current Limits**: Set appropriate current limits to prevent brownouts

## Additional Resources
- [CTRE Phoenix 6 Documentation](https://v6.docs.ctr-electronics.com/)
- [Migration Guide from Phoenix 5](https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/)
- [Hardware Reference](https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/)