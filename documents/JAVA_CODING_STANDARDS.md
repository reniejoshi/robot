# Java Coding Standards for FRC Robot Development

## Overview

This document establishes coding standards for the Bear Metal FRC robot codebase. These standards ensure code consistency, readability, maintainability, and team collaboration effectiveness.

## General Principles

### Code Quality Goals
1. **Readability**: Code should be self-documenting and easy to understand
2. **Consistency**: Uniform style across the entire codebase
3. **Maintainability**: Easy to modify and extend
4. **Performance**: Efficient for real-time robot control
5. **Safety**: Fail-safe behaviors and proper error handling

### Design Philosophy
- **Composition over Inheritance**: Favor object composition
- **Single Responsibility**: Each class should have one reason to change
- **Dependency Injection**: Use constructor injection for testability
- **Immutability**: Prefer immutable objects where possible
- **Fail-Fast**: Detect errors early and report them clearly

## File Organization

### Package Structure
```java
org.tahomarobotics.
├── auto/              // Autonomous routines and constants
├── chassis/           // Drivetrain subsystem
├── collector/         // Game piece collection
├── grabber/          // Game piece manipulation
├── windmill/         // Arm and elevator system
├── climber/          // End-game climbing
├── vision/           // Camera and localization
├── sim/              // Simulation classes
└── util/             // Utility classes and interfaces
```

### File Naming
- **Classes**: PascalCase (e.g., `DriveSubsystem`, `VisionProcessor`)
- **Interfaces**: PascalCase, no prefix (e.g., `Localizer`, `PathFollower`)
- **Enums**: PascalCase (e.g., `AllianceColor`, `GamePiece`)
- **Files**: Match class name exactly

### File Header
All source files must include the MIT license header:

```java
/*
 * MIT License
 *
 * Copyright (c) 2025 Bear Metal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
```

## Import Organization

### Import Order
1. Java standard library (`java.*`)
2. Third-party libraries (alphabetical by root package)
3. WPILib (`edu.wpi.first.*`)
4. CTRE Phoenix (`com.ctre.phoenix6.*`)
5. REV Robotics (`com.revrobotics.*`)
6. Other FRC vendors
7. Project imports (`org.tahomarobotics.*`)

### Import Rules
- **No wildcard imports** except for static imports of constants
- **Static imports** only for frequently used constants and units
- **Unused imports** must be removed

```java
// Good - specific imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

// Good - static imports for constants and units
import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.chassis.ChassisConstants.*;

// Bad - wildcard imports
import edu.wpi.first.wpilibj2.command.*;
```

## Naming Conventions

### Variables and Methods
- **camelCase** for variables, methods, and parameters
- **Descriptive names** that explain purpose
- **No abbreviations** unless widely understood in FRC context

```java
// Good
private final TalonFX leftDriveMotor;
private double targetVelocityMetersPerSecond;
public void setDesiredTrajectory(Trajectory trajectory) { }

// Bad
private final TalonFX lDrvMtr;
private double tgtVelMPS;
public void setTraj(Trajectory t) { }
```

### Constants
- **SCREAMING_SNAKE_CASE** for constants
- **Group related constants** in static nested classes
- **Use Units library** for all physical quantities

```java
public class ChassisConstants {
    // Hardware IDs
    public static final int LEFT_FRONT_DRIVE_ID = 1;
    public static final int LEFT_FRONT_STEER_ID = 2;
    
    // Physical constants with units
    public static final Distance WHEELBASE = Meters.of(0.62);
    public static final Velocity MAX_VELOCITY = MetersPerSecond.of(4.5);
    
    // PID gains grouped in nested class
    public static final class DriveGains {
        public static final double KP = 0.1;
        public static final double KI = 0.0;
        public static final double KD = 0.01;
        public static final double KS = 0.25;
        public static final double KV = 0.12;
    }
}
```

### Class Members
- **Private fields** with descriptive names
- **final** for immutable fields
- **Group fields** by purpose with blank lines

```java
public class DriveSubsystem extends AbstractSubsystem {
    // Hardware
    private final TalonFX leftFrontDrive;
    private final TalonFX leftFrontSteer;
    private final CANCoder leftFrontEncoder;
    
    // Status signals
    private final StatusSignal<Angle> leftFrontPosition;
    private final StatusSignal<AngularVelocity> leftFrontVelocity;
    
    // Control requests
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0);
    
    // State
    private SwerveModuleState desiredState = new SwerveModuleState();
}
```

## Code Formatting

### Indentation and Spacing
- **4 spaces** for indentation (no tabs)
- **Maximum line length**: 100 characters
- **Blank line** after class declaration, between methods, between logical blocks

### Brace Style
- **Opening brace** on same line
- **Closing brace** on new line, aligned with statement
- **Always use braces** even for single statements

```java
// Good
public void drive(double speed) {
    if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
    }
    
    motor.set(speed);
}

// Bad
public void drive(double speed) 
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;
    motor.set(speed);
}
```

### Method Parameters
- **One parameter per line** if line exceeds 100 characters
- **Align parameters** with first parameter

```java
// Good - fits on one line
public void configureMotor(TalonFX motor, TalonFXConfiguration config) {

// Good - multiple lines when needed
public void configureSwerveModule(TalonFX driveMotor,
                                  TalonFX steerMotor,
                                  CANCoder steerEncoder,
                                  SwerveModuleConstants constants) {
```

## Constructor Patterns

### Standard Pattern
Every subsystem should have exactly two constructors:

```java
public class ExampleSubsystem extends AbstractSubsystem {
    private final TalonFX motor;
    private final DigitalInput limitSwitch;
    
    // Default constructor - only place that creates hardware (except tests)
    public ExampleSubsystem() {
        motor = new TalonFX(MOTOR_ID, CANIVORE_NAME);
        limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
        configureHardware();
    }
    
    // Package-private constructor for unit testing
    ExampleSubsystem(ExampleSubsystem other) {
        this.motor = other.motor;
        this.limitSwitch = other.limitSwitch;
    }
    
    private void configureHardware() {
        // Hardware configuration logic
    }
}
```

## Method Organization

### Method Order
1. **Constructors**
2. **Public methods** (grouped by functionality)
3. **Package-private methods**
4. **Private methods**
5. **Static methods**
6. **Overridden methods** (grouped together)

### Method Length
- **Maximum 50 lines** per method
- **Extract complex logic** into private helper methods
- **Single responsibility** per method

```java
// Good - focused method
public void setDesiredState(SwerveModuleState desiredState) {
    this.desiredState = optimizeModuleState(desiredState);
    applyModuleState();
}

private SwerveModuleState optimizeModuleState(SwerveModuleState state) {
    // Optimization logic
}

private void applyModuleState() {
    // Apply the state to hardware
}
```

## Error Handling

### Exception Handling
- **Catch specific exceptions** rather than generic Exception
- **Log errors** using TinyLog, not System.out.println
- **Fail gracefully** with safe default behaviors

```java
// Good
public void configureMotor() {
    try {
        motor.getConfigurator().apply(motorConfig, TIMEOUT_SECONDS);
        Logger.tag("DriveSubsystem").info("Motor configured successfully");
    } catch (TimeoutException e) {
        Logger.tag("DriveSubsystem").error(e, "Motor configuration timeout");
        // Set safe default state
        motor.stopMotor();
    }
}

// Bad
public void configureMotor() {
    try {
        motor.getConfigurator().apply(motorConfig);
    } catch (Exception e) {
        System.out.println("Error: " + e.getMessage());
    }
}
```

### Parameter Validation
- **Validate parameters** in public methods
- **Use Objects.requireNonNull()** for null checks
- **Document preconditions** in method comments when needed

```java
public void setTrajectory(Trajectory trajectory) {
    Objects.requireNonNull(trajectory, "Trajectory cannot be null");
    
    if (trajectory.getTotalTimeSeconds() <= 0) {
        throw new IllegalArgumentException("Trajectory duration must be positive");
    }
    
    this.currentTrajectory = trajectory;
}
```

## Documentation Standards

### JavaDoc Requirements
- **All public classes** must have class-level JavaDoc
- **All public methods** must have method-level JavaDoc
- **Complex private methods** should have JavaDoc

```java
/**
 * Controls a swerve drive module with independent steering and drive motors.
 * Provides closed-loop control for both position and velocity.
 * 
 * <p>This class handles the low-level motor control and sensor feedback
 * for a single swerve module, including angle optimization and
 * continuous rotation handling.
 */
public class SwerveModule {
    
    /**
     * Sets the desired state for this swerve module.
     * 
     * <p>The angle will be optimized to minimize rotation distance,
     * potentially reversing the drive direction to avoid rotating
     * more than 90 degrees.
     * 
     * @param desiredState the target state for this module
     * @throws IllegalArgumentException if desiredState is null
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Implementation
    }
}
```

### Inline Comments
- **Explain why**, not what
- **Use for complex algorithms** and business logic
- **Avoid obvious comments**

```java
// Good - explains why
// Reverse drive direction to avoid rotating more than 90 degrees
if (Math.abs(angleError) > Math.PI / 2) {
    desiredState.speedMetersPerSecond *= -1;
    desiredState.angle = desiredState.angle.rotateBy(Rotation2d.fromRadians(Math.PI));
}

// Bad - explains what (obvious from code)
// Set the speed to the desired speed
motor.set(desiredSpeed);
```

## Testing Standards

### Test Class Organization
- **One test class per production class**
- **Test class name**: `{ClassName}Test`
- **Package structure**: Mirror production code

```java
@Tag("unit")
public class SwerveModuleTest extends BaseTest {
    
    @Test
    public void testAngleOptimization() {
        // Given
        SwerveModule module = new SwerveModule();
        SwerveModuleState initialState = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0));
        
        // When
        module.setDesiredState(new SwerveModuleState(1.0, Rotation2d.fromDegrees(170)));
        
        // Then
        SwerveModuleState actualState = module.getState();
        assertEquals(-1.0, actualState.speedMetersPerSecond, 0.01);
        assertEquals(-10.0, actualState.angle.getDegrees(), 0.01);
    }
}
```

### Test Method Naming
- **Descriptive names** that explain the scenario
- **Format**: `test{Scenario}` or `should{ExpectedBehavior}When{Condition}`

```java
@Test
public void testAngleOptimizationWithLargeRotation() { }

@Test
public void shouldReverseDirectionWhenAngleChangeExceeds90Degrees() { }
```

## Performance Guidelines

### Object Creation
- **Minimize object creation** in periodic methods
- **Reuse objects** like control requests and mathematical objects
- **Use object pooling** for frequently created objects

```java
public class DriveSubsystem extends AbstractSubsystem {
    // Reuse control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    
    // Reuse mathematical objects
    private final Translation2d tempTranslation = new Translation2d();
    
    public void drive(double voltage) {
        // Good - reuse existing object
        motor.setControl(voltageRequest.withOutput(voltage));
        
        // Bad - creates new object every call
        motor.setControl(new VoltageOut(voltage));
    }
}
```

### CAN Bus Optimization
- **Group status signals** by update frequency
- **Use BaseStatusSignal.setUpdateFrequencyForAll()**
- **Optimize bus utilization** with ParentDevice.optimizeBusUtilizationForAll()

```java
public void configureSignals() {
    // Critical control signals - high frequency
    BaseStatusSignal.setUpdateFrequencyForAll(100.0,
        leftDrivePosition, leftDriveVelocity,
        rightDrivePosition, rightDriveVelocity);
    
    // Diagnostic signals - lower frequency
    BaseStatusSignal.setUpdateFrequencyForAll(10.0,
        leftDriveVoltage, leftDriveCurrent,
        rightDriveVoltage, rightDriveCurrent);
    
    // Optimize overall bus usage
    ParentDevice.optimizeBusUtilizationForAll(leftDrive, rightDrive);
}
```

## Common Anti-Patterns to Avoid

### Code Smells
1. **God Classes**: Classes that do too many things
2. **Long Parameter Lists**: More than 4-5 parameters
3. **Magic Numbers**: Unexplained numeric constants
4. **Duplicate Code**: Copy-pasted logic
5. **Deep Nesting**: More than 3 levels of if/for statements

### FRC-Specific Anti-Patterns
1. **Creating hardware objects in periodic methods**
2. **Using System.out.println instead of proper logging**
3. **Not handling CAN timeouts gracefully**
4. **Blocking operations in periodic methods**
5. **Not optimizing CAN bus usage**

```java
// Bad - creates objects in periodic
@Override
public void periodic() {
    motor.setControl(new VoltageOut(12.0)); // Creates new object each call
    System.out.println("Motor voltage: " + motor.getMotorVoltage()); // Wrong logging
}

// Good - reuses objects and proper logging
private final VoltageOut voltageRequest = new VoltageOut(0);

@Override
public void periodic() {
    motor.setControl(voltageRequest.withOutput(12.0));
    Logger.tag("Drive").debug("Motor voltage: {}", motor.getMotorVoltage());
}
```

## Tool Integration

### IntelliJ IDEA Settings
- **Code Style**: Import team settings from `.idea/codeStyles/`
- **Inspections**: Enable all recommended warnings
- **Auto-format**: Ctrl+Alt+L before committing
- **Optimize imports**: Ctrl+Alt+O to remove unused imports

### Gradle Build
- **Checkstyle**: Enforce coding standards automatically
- **SpotBugs**: Static analysis for bug detection
- **JaCoCo**: Code coverage reporting

```gradle
// Applied in build.gradle
plugins {
    id 'checkstyle'
    id 'com.github.spotbugs'
    id 'jacoco'
}

checkstyle {
    configFile = file('config/checkstyle/checkstyle.xml')
    ignoreFailures = false
}
```

## Git and Version Control

### Commit Standards
- **Atomic commits**: One logical change per commit
- **Descriptive messages**: Explain what and why, not how
- **Format**: `Type: Short description\n\nLonger explanation if needed`

```
feat: Add swerve drive auto-alignment command

Implements automatic alignment to AprilTags using PhotonVision
for improved autonomous scoring accuracy. Includes PID control
for both translation and rotation.
```

### Code Review Checklist
- [ ] Code follows naming conventions
- [ ] All public methods have JavaDoc
- [ ] No magic numbers or hardcoded values
- [ ] Error handling is appropriate
- [ ] Performance considerations addressed
- [ ] Unit tests included for new functionality
- [ ] No violations of coding standards

## Conclusion

These standards ensure our codebase remains maintainable, readable, and performant throughout the FRC season. When in doubt, prioritize code clarity and team understanding over cleverness or premature optimization.

For questions or clarifications about these standards, consult with senior team members or refer to the existing codebase for examples of proper implementation.