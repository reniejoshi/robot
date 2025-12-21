# New Subsystem Creation Guide

## Overview
This guide provides a step-by-step process for creating new subsystems following the established architectural patterns in this FRC robot codebase. All subsystems follow consistent naming conventions, package structure, and integration patterns with AdvantageKit logging.

## Subsystem Architecture Pattern

### Core Classes Structure
Every subsystem follows this 4-class pattern:

```
src/main/java/org/tahomarobotics/robot/{subsystemname}/
├── {SubsystemName}Constants.java    // Hardware constants and configuration
├── {SubsystemName}Subsystem.java    // WPILib subsystem (extends AbstractSubsystem)
├── {SubsystemName}.java             // Command factory and high-level interface
└── {SubsystemName}Simulation.java   // Physics simulation (extends AbstractSimulation)
```


## Step-by-Step Creation Process

### Step 0: Hardware Specification Gathering

**Before writing any code, gather this information about your mechanism:**

#### Required Hardware Information
Use this checklist to collect all necessary details:

**📋 Mechanism Overview**
- [ ] **Mechanism Name**: What is this subsystem called? (e.g., "Shooter", "Intake", "Elevator")
- [ ] **Primary Function**: What does it do? (e.g., "Shoots game pieces", "Collects notes", "Lifts robot")
- [ ] **Movement Type**: Linear or rotational movement?
- [ ] **Range of Motion**: How far does it move? (degrees, inches, etc.)

**🔧 Motor Information**
- [ ] **Number of Motors**: How many motors control this mechanism?
- [ ] **Motor Type**: What type of motors? (TalonFX/Kraken X60, NEO, CIM, etc.)
- [ ] **Motor CAN IDs**: What are the CAN IDs for each motor? (e.g., 10, 11, 12)
- [ ] **Motor Pairing**: Are any motors paired/follower configuration?
- [ ] **Motor Direction**: Which direction is "positive" for each motor?
- [ ] **Current Limits**: What current limits are safe? (typically 40A for mechanisms)

**⚙️ Gearing and Mechanical**
- [ ] **Gear Ratio**: What is the total gear reduction? (e.g., 10:1, 50:1)
- [ ] **Mechanism Mass**: Approximate weight of moving parts (kg)
- [ ] **Mechanism Dimensions**: Length, radius, or other relevant dimensions

**📡 Sensors**
- [ ] **Encoders**: Built-in motor encoders or external encoders?
- [ ] **Limit Switches**: Digital inputs for end-of-travel? (DIO port numbers)
- [ ] **Other Sensors**: Beam breaks, potentiometers, gyros, etc.
- [ ] **Sensor Mounting**: Where are sensors physically located?

**🎯 Control Requirements**
- [ ] **Precision Needed**: High precision (±1°) or basic control (±5°)?
- [ ] **Speed Requirements**: Fast movement or slow/controlled?
- [ ] **Safety Limits**: Soft limits, hard stops, emergency stops?
- [ ] **Typical Setpoints**: What positions/speeds will it commonly use?

#### Example Hardware Specification

**Shooter Subsystem Example:**
```yaml
Mechanism Name: "Shooter"
Primary Function: "Launch game pieces at high velocity"
Movement Type: "Rotational (flywheel)"
Range of Motion: "0 to 6000 RPM"

Motors:
- Count: 2 motors
- Type: TalonFX (Kraken X60)
- CAN IDs: [15, 16]
- Pairing: Motor 16 follows Motor 15 (opposite direction)
- Direction: Positive = shooting direction
- Current Limit: 60A (high for shooter)

Gearing:
- Gear Ratio: 1:1 (direct drive)
- Flywheel Mass: 3 kg
- Flywheel Radius: 0.1 meters

Sensors:
- Encoders: Built-in TalonFX encoders
- Beam Break: DIO port 3 (game piece detection)
- No limit switches needed

Control:
- Precision: ±50 RPM
- Speed: Fast response required
- Setpoints: 3000 RPM, 4500 RPM, 6000 RPM
```

**Intake Subsystem Example:**
```yaml
Mechanism Name: "Intake" 
Primary Function: "Collect game pieces from floor"
Movement Type: "Rotational (roller) + Linear (deploy/retract)"
Range of Motion: "Roller: continuous, Deploy: 0-90 degrees"

Motors:
- Count: 2 motors
- Type: 1x TalonFX (roller), 1x NEO 550 (deploy)
- CAN IDs: [12 (roller), 13 (deploy)]
- Pairing: No followers
- Direction: Positive roller = intake, Positive deploy = extend
- Current Limit: 40A (roller), 20A (deploy)

Gearing:
- Roller Ratio: 3:1 reduction
- Deploy Ratio: 100:1 reduction  
- Intake Mass: 2 kg
- Deploy Arm Length: 0.3 meters

Sensors:
- Encoders: Built-in motor encoders
- Limit Switches: DIO 4 (fully retracted), DIO 5 (fully deployed)
- Beam Break: DIO 6 (game piece collected)

Control:
- Precision: ±2 degrees (deploy), ±100 RPM (roller)
- Speed: Medium response
- Setpoints: Retracted (0°), Deployed (85°), Roller (2000 RPM)
```

#### Interactive Hardware Specification Questions

**When working with a student, ask these questions in order:**

1. **"What does your mechanism do?"** → Determines subsystem purpose
2. **"Does it spin, slide, or both?"** → Movement type
3. **"How many motors control it?"** → Motor count  
4. **"What type of motors are they?"** → Motor type selection
5. **"What are their CAN IDs?"** → Hardware mapping
6. **"Do any motors work together?"** → Follower relationships
7. **"Which way is 'positive' movement?"** → Direction conventions
8. **"How much reduction gearing?"** → Gear ratios
9. **"What sensors tell you where it is?"** → Feedback devices
10. **"Where does it need to stop?"** → Limits and setpoints
11. **"How precisely must it move?"** → Control requirements
12. **"How fast should it respond?"** → Performance needs

#### Hardware Specification to Code Mapping

**Once you have the hardware specification, here's how it maps to code:**

| Hardware Info | Goes Into | Code Example |
|---------------|-----------|--------------|
| CAN IDs | Constants class | `public static final int MOTOR_ID = 15;` |
| Motor type | Import statements | `import com.ctre.phoenix6.hardware.TalonFX;` |
| Gear ratio | Constants class | `public static final double GEAR_RATIO = 10.0;` |
| Current limits | Motor config | `config.CurrentLimits.StatorCurrentLimit = 40;` |
| Motor direction | Motor config | `config.MotorOutput.Inverted = CounterClockwise_Positive;` |
| DIO ports | Hardware declarations | `private final DigitalInput limitSwitch = new DigitalInput(4);` |
| Setpoints | Constants class | `public static final Angle DEPLOYED = Rotations.of(0.25);` |
| Control precision | PID gains | `public static final double KP = 2.4;` |
| Mechanism mass | Simulation | `private static final double MASS = 2.0; // kg` |
| Movement type | Control requests | `MotionMagicVoltage` vs `VelocityVoltage` |

#### Common Hardware Configurations

**Single Motor with Encoder:**
```java
// Constants
public static final int MOTOR_ID = 10;
public static final double GEAR_RATIO = 15.0;

// Subsystem
private final TalonFX motor = new TalonFX(MOTOR_ID, RobotMap.CANIVORE_NAME);
```

**Dual Motor with Follower:**
```java
// Constants  
public static final int LEADER_ID = 10;
public static final int FOLLOWER_ID = 11;

// Subsystem
private final TalonFX leader = new TalonFX(LEADER_ID, RobotMap.CANIVORE_NAME);
private final TalonFX follower = new TalonFX(FOLLOWER_ID, RobotMap.CANIVORE_NAME);

// Constructor
follower.setControl(new Follower(leader.getDeviceID(), true)); // true = oppose leader
```

**Motor with Limit Switches:**
```java
// Constants
public static final int RETRACTED_LIMIT_DIO = 4;
public static final int EXTENDED_LIMIT_DIO = 5;

// Subsystem  
private final DigitalInput retractedLimit = new DigitalInput(RETRACTED_LIMIT_DIO);
private final DigitalInput extendedLimit = new DigitalInput(EXTENDED_LIMIT_DIO);

// Usage
public boolean isFullyRetracted() {
    return !retractedLimit.get(); // Inverted for normally open
}
```

**Motor with Beam Break Sensor:**
```java
// Constants
public static final int BEAM_BREAK_DIO = 6;

// Subsystem
private final DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_DIO);

// Usage  
public boolean hasGamePiece() {
    return !beamBreak.get(); // Inverted - beam blocked = game piece present
}
```

#### Common FRC Mechanism Types

**For students unsure about their mechanism, here are typical FRC subsystems:**

| Mechanism Type | Description | Typical Hardware | Common Setpoints |
|----------------|-------------|------------------|-------------------|
| **Shooter** | Launch game pieces | 1-2 TalonFX motors, flywheel | 3000, 4500, 6000 RPM |
| **Intake** | Collect game pieces | 1 TalonFX (roller) + 1 NEO (deploy) | Deploy: 0°/90°, Roller: 2000 RPM |
| **Elevator** | Lift vertically | 1-2 TalonFX motors, gearbox | Ground: 0", Low: 12", High: 36" |  
| **Arm** | Rotate horizontally | 1-2 TalonFX motors, gearbox | Stowed: 0°, Score: 45°, Intake: -15° |
| **Climber** | End-game climbing | 1-2 TalonFX motors, winch | Retracted: 0", Extended: 24" |
| **Wrist** | Orient end effector | 1 NEO motor, high reduction | Horizontal: 0°, Down: -90°, Up: 90° |

**Questions to determine mechanism type:**
- "Does it shoot something?" → **Shooter**
- "Does it pick things up?" → **Intake** 
- "Does it go up and down?" → **Elevator**
- "Does it rotate/swing?" → **Arm** or **Wrist**
- "Does it lift the robot?" → **Climber**

### Step 1: Create Package Directory
```bash
# Create the subsystem package directory
mkdir -p src/main/java/org/tahomarobotics/robot/{subsystemname}
```

**Example for "intake" subsystem:**
```bash
mkdir -p src/main/java/org/tahomarobotics/robot/intake
```

### Step 2: Create Constants Class

**File:** `{SubsystemName}Constants.java`

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

package org.tahomarobotics.robot.{subsystemname};

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class {SubsystemName}Constants {
    
    // Hardware IDs
    public static final int MOTOR_ID = 10; // Update with actual CAN ID
    public static final int SENSOR_DIO_PORT = 0; // Digital input port
    
    // Physical Constants
    public static final double GEAR_RATIO = 10.0;
    public static final Voltage MAX_VOLTAGE = Volts.of(12.0);
    public static final Current CURRENT_LIMIT = Amps.of(40.0);
    
    // Control Constants
    public static final double KS = 0.25; // Static friction (V)
    public static final double KV = 0.12; // Velocity gain (V·s/rot)  
    public static final double KA = 0.01; // Acceleration gain (V·s²/rot)
    public static final double KP = 2.4;  // Proportional gain
    public static final double KI = 0.0;  // Integral gain
    public static final double KD = 0.1;  // Derivative gain
    
    // Setpoints and Limits
    public static final Angle MAX_POSITION = Rotations.of(5.0);
    public static final Angle MIN_POSITION = Rotations.of(0.0);
    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(20.0);
    
    // Configuration Objects
    public static TalonFXConfiguration createMotorConfig() {
        var config = new TalonFXConfiguration();
        
        // Motor output
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Current limiting
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Slot 0 gains
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kP = KP;
        config.Slot0.kI = KI;
        config.Slot0.kD = KD;
        
        return config;
    }
    
    // Control Requests (reusable objects)
    public static final class ControlRequests {
        public static final VoltageOut voltageRequest = new VoltageOut(0);
        public static final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    }
}
```

### Step 3: Create Subsystem Class

**File:** `{SubsystemName}Subsystem.java`

```java
/*
 * MIT License
 *
 * Copyright (c) 2025 Bear Metal
 * [... same license header as above ...]
 */

package org.tahomarobotics.robot.{subsystemname};

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.robot.{subsystemname}.{SubsystemName}Constants.*;

public class {SubsystemName}Subsystem extends AbstractSubsystem implements AutoCloseable {
    
    // Hardware
    private final TalonFX motor;
    private final DigitalInput limitSwitch;
    
    // Status signals for efficient CAN usage
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    
    // Control requests (reused for performance)
    private final VoltageOut voltageRequest = ControlRequests.voltageRequest;
    
    // Default constructor - only place that creates the subsystem (except unit tests)
    public {SubsystemName}Subsystem() {
        motor = new TalonFX(MOTOR_ID, RobotMap.CANIVORE_NAME);
        limitSwitch = new DigitalInput(SENSOR_DIO_PORT);
        
        // Configure motor
        RobustConfigurator.configure(motor, createMotorConfig());
        
        // Initialize status signals
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        currentSignal = motor.getStatorCurrent();
        
        // Optimize CAN usage
        BaseStatusSignal.setUpdateFrequencyForAll(50, positionSignal, velocitySignal);
        BaseStatusSignal.setUpdateFrequencyForAll(10, voltageSignal, currentSignal);
        ParentDevice.optimizeBusUtilizationForAll(motor);
    }
    
    // Package-private constructor for unit testing
    {SubsystemName}Subsystem({SubsystemName}Subsystem subsystem) {
        this.motor = subsystem.motor;
        this.limitSwitch = subsystem.limitSwitch;
        this.positionSignal = subsystem.positionSignal;
        this.velocitySignal = subsystem.velocitySignal;
        this.voltageSignal = subsystem.voltageSignal;
        this.currentSignal = subsystem.currentSignal;
    }
    
    @Override
    public void subsystemPeriodic() {
        // Refresh status signals
        BaseStatusSignal.refreshAll(positionSignal, velocitySignal, voltageSignal, currentSignal);
        
        // Log data with AdvantageKit
        Logger.recordOutput("{SubsystemName}/Position", positionSignal.getValueAsDouble());
        Logger.recordOutput("{SubsystemName}/Velocity", velocitySignal.getValueAsDouble());
        Logger.recordOutput("{SubsystemName}/Voltage", voltageSignal.getValueAsDouble());
        Logger.recordOutput("{SubsystemName}/Current", currentSignal.getValueAsDouble());
        Logger.recordOutput("{SubsystemName}/LimitSwitch", getLimitSwitch());
        
        // Log motor temperature if needed
        if (Robot.isReal()) {
            Logger.recordOutput("{SubsystemName}/Temperature", motor.getDeviceTemp().getValueAsDouble());
        }
    }
    
    // Public interface methods
    public void setVoltage(Voltage voltage) {
        motor.setControl(voltageRequest.withOutput(voltage.in(Volts)));
    }
    
    public void stop() {
        setVoltage(Volts.of(0.0));
    }
    
    public Angle getPosition() {
        return Rotations.of(positionSignal.getValueAsDouble());
    }
    
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(velocitySignal.getValueAsDouble());
    }
    
    public boolean getLimitSwitch() {
        return !limitSwitch.get(); // Inverted for normally open switch
    }
    
    // Supplier methods for command factories
    public BooleanSupplier limitSwitchSupplier() {
        return this::getLimitSwitch;
    }
    
    public BooleanSupplier atPosition(Angle targetPosition, Angle tolerance) {
        return () -> Math.abs(getPosition().in(Rotations) - targetPosition.in(Rotations)) < tolerance.in(Rotations);
    }
    
    @Override
    public void close() {
        motor.close();
        limitSwitch.close();
    }
}
```

### Step 4: Create Command Factory Class

**File:** `{SubsystemName}.java`

```java
/*
 * MIT License
 *
 * Copyright (c) 2025 Bear Metal
 * [... same license header as above ...]
 */

package org.tahomarobotics.robot.{subsystemname};

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import org.tahomarobotics.robot.util.CommandLogger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class {SubsystemName} implements AutoCloseable {
    
    // Subsystem reference
    private final {SubsystemName}Subsystem subsystem;
    
    // Default constructor - creates new subsystem instance
    public {SubsystemName}() {
        this(new {SubsystemName}Subsystem());
    }
    
    // Package-private constructor for unit testing
    {SubsystemName}({SubsystemName}Subsystem subsystem) {
        this.subsystem = subsystem;
    }
    
    // Basic commands
    public Command runVoltage(DoubleSupplier voltageSupplier) {
        return subsystem.run(() -> {
            double voltage = voltageSupplier.getAsDouble();
            subsystem.setVoltage(Volts.of(voltage));
        }).withName("{SubsystemName} Run Voltage");
    }
    
    public Command stop() {
        return subsystem.runOnce(() -> subsystem.stop())
            .withName("{SubsystemName} Stop");
    }
    
    // Complex command example
    public Command moveToPosition(double targetRotations) {
        return new FunctionalCommand(
            // onInit
            () -> {
                CommandLogger.logCommandStart("Move {SubsystemName} to " + targetRotations);
                // TODO: Implement position control
            },
            // onExecute  
            () -> {
                // TODO: Implement position control loop
                // This would typically use Motion Magic or PID control
            },
            // onEnd
            (interrupted) -> {
                subsystem.stop();
                CommandLogger.logCommandEnd("Move {SubsystemName}", interrupted);
            },
            // isFinished
            () -> subsystem.atPosition(Rotations.of(targetRotations), Rotations.of(0.1)).getAsBoolean(),
            // requirements
            subsystem
        ).withName("{SubsystemName} Move to Position");
    }
    
    // Convenience methods for common operations
    public Command deploy() {
        return moveToPosition({SubsystemName}Constants.MAX_POSITION.in(Rotations));
    }
    
    public Command retract() {
        return moveToPosition({SubsystemName}Constants.MIN_POSITION.in(Rotations));
    }
    
    @Override
    public void close() throws Exception {
        subsystem.close();
    }
}
```

### Step 5: Create Simulation Class

**File:** `{SubsystemName}Simulation.java`

```java
/*
 * MIT License
 *
 * Copyright (c) 2025 Bear Metal
 * [... same license header as above ...]
 */

package org.tahomarobotics.robot.{subsystemname};

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import org.tahomarobotics.robot.sim.AbstractSimulation;

import static edu.wpi.first.units.Units.*;

public class {SubsystemName}Simulation extends AbstractSimulation {
    
    // Physics simulation
    private final LinearSystemSim<N2, N1, N1> simulation;
    private final {SubsystemName}Subsystem subsystem;
    
    // Simulation parameters
    private static final double MECHANISM_MASS = 2.0; // kg
    private static final double MECHANISM_LENGTH = 0.5; // meters
    private static final DCMotor MOTOR = DCMotor.getFalcon500(1);
    
    public {SubsystemName}Simulation({SubsystemName}Subsystem subsystem) {
        this.subsystem = subsystem;
        
        // Create physics plant model
        LinearSystem<N2, N1, N1> plant = LinearSystemId.createSingleJointedArmSystem(
            MOTOR, 
            MECHANISM_MASS * MECHANISM_LENGTH * MECHANISM_LENGTH / 3.0, // J (moment of inertia)
            {SubsystemName}Constants.GEAR_RATIO
        );
        
        this.simulation = new LinearSystemSim<>(plant);
    }
    
    @Override
    public void updateSim() {
        // Get motor voltage from subsystem
        double motorVoltage = subsystem.voltageSignal.getValueAsDouble();
        
        // Set input voltage to simulation
        simulation.setInput(motorVoltage);
        
        // Update simulation
        simulation.update(0.020); // 20ms robot period
        
        // Apply simulated position and velocity back to motor
        double position = simulation.getOutput(0); // Position in rotations
        double velocity = simulation.getOutput(1); // Velocity in rotations/sec
        
        // Update motor simulation state
        subsystem.motor.getSimState().setRawRotorPosition(position);
        subsystem.motor.getSimState().setRotorVelocity(velocity);
        
        // Simulate limit switch
        boolean limitHit = position <= {SubsystemName}Constants.MIN_POSITION.in(Rotations);
        // Note: Limit switch simulation would need additional implementation
    }
    
    // Helper methods for testing
    public double getSimulatedPosition() {
        return simulation.getOutput(0);
    }
    
    public double getSimulatedVelocity() {
        return simulation.getOutput(1);
    }
}
```

### Step 6: Integration with RobotContainer

**Add to RobotContainer.java:**

```java
// Add to imports
import org.tahomarobotics.robot.{subsystemname}.{SubsystemName};
import org.tahomarobotics.robot.{subsystemname}.{SubsystemName}Simulation;
import org.tahomarobotics.robot.{subsystemname}.{SubsystemName}Subsystem;

// Add fields
public final {SubsystemName} {subsystemname};

// Add to constructor
{subsystemname} = new {SubsystemName}();

// Add to simulation section (if Robot.isSimulation())
simulations.add(new {SubsystemName}Simulation({subsystemname}.subsystem));

// Add to close() method
{subsystemname}.close();
```

### Step 7: Add to RobotMap (if using custom CAN bus)

**Add to RobotMap.java:**
```java
// Add CAN IDs section for your subsystem
public static final class {SUBSYSTEM_NAME}_IDS {
    public static final int MOTOR = 10; // Update with actual ID
}
```

## Testing Your New Subsystem

### Step 8: Create Basic Test

**File:** `src/test/java/org/tahomarobotics/robot/{subsystemname}/{SubsystemName}Test.java`

```java
package org.tahomarobotics.robot.{subsystemname};

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.tahomarobotics.robot.BaseTest;

import static org.junit.jupiter.api.Assertions.*;

@Tag("unit")
public class {SubsystemName}Test extends BaseTest {
    
    @Test
    public void testSubsystemCreation() {
        // Create subsystem using default constructor
        {SubsystemName}Subsystem originalSubsystem = new {SubsystemName}Subsystem();
        assertNotNull(originalSubsystem);
        
        // Test package-private constructor for unit testing
        {SubsystemName}Subsystem testSubsystem = new {SubsystemName}Subsystem(originalSubsystem);
        assertNotNull(testSubsystem);
        
        // Test basic functionality
        testSubsystem.stop();
        assertEquals(0.0, testSubsystem.getVelocity().in(RotationsPerSecond), 0.01);
        
        originalSubsystem.close();
    }
}
```

## Step 6: SysId Integration for Motor Characterization

Before finalizing your subsystem, set up SysId to characterize motors and tune feedforward/feedback gains. This is essential for achieving optimal performance.

### SysId Routine Class Template

Create `{SubsystemName}SysIdRoutine.java`:

```java
/*
 * MIT License
 * [License header...]
 */

package org.tahomarobotics.robot.{subsystemname};

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.*;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class {SubsystemName}SysIdRoutine {
    
    private final {SubsystemName}Subsystem subsystem;
    private final SysIdRoutine sysIdRoutine;
    
    public {SubsystemName}SysIdRoutine({SubsystemName}Subsystem subsystem) {
        this.subsystem = subsystem;
        
        // Configure SysId routine
        this.sysIdRoutine = new SysIdRoutine(
            new Config(
                // Ramp rate - start conservative, increase if needed
                Volts.of(1).per(Seconds.of(1)), // 1 volt per second
                
                // Maximum voltage - match your mechanism's safe operating voltage
                Volts.of(7),  // Start conservative, increase to 10-12V if needed
                
                // Timeout for each test
                Seconds.of(10),
                
                // Data recording callback
                (state) -> Logger.recordOutput("SysId/State", state.toString())
            ),
            new Mechanism(
                // Drive function - applies voltage to motor
                (Measure<Voltage> volts) -> subsystem.setVoltage(volts.in(Volts)),
                
                // Log function - records motor data
                (SysIdRoutineLog log) -> {
                    // Log motor voltage (input)
                    log.motor("motor")
                        .voltage(Volts.of(subsystem.getAppliedVoltage()))
                        .position(Rotations.of(subsystem.getPosition().in(Rotations)))
                        .velocity(RotationsPerSecond.of(subsystem.getVelocity().in(RotationsPerSecond)));
                },
                
                subsystem // Subsystem requirement
            )
        );
    }
    
    // Quasistatic tests - slow ramp to measure kS and kV
    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }
    
    public Command quasistaticReverse() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }
    
    // Dynamic tests - quick step to measure kA
    public Command dynamicForward() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }
    
    public Command dynamicReverse() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }
}
```

### Required Subsystem Methods

Add these methods to your `{SubsystemName}Subsystem.java`:

```java
/**
 * Sets motor voltage directly for SysId characterization.
 * @param volts voltage to apply to motor
 */
public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
}

/**
 * Gets the applied voltage for SysId logging.
 * @return applied voltage in volts
 */
public double getAppliedVoltage() {
    return voltageSignal.getValueAsDouble();
}

/**
 * Gets current motor position for SysId.
 * @return position in rotations (mechanism rotations, not motor rotations)
 */
public Measure<Angle> getPosition() {
    return Rotations.of(positionSignal.getValueAsDouble());
}

/**
 * Gets current motor velocity for SysId.
 * @return velocity in rotations per second (mechanism RPM, not motor RPM)
 */
public Measure<AngularVelocity> getVelocity() {
    return RotationsPerSecond.of(velocitySignal.getValueAsDouble());
}

/**
 * Stops all motor movement - used by SysId for safety.
 */
public void stop() {
    setVoltage(Volts.of(0.0));
}
```

### RobotContainer Integration

Add SysId commands to `RobotContainer.java`:

```java
// In RobotContainer class
private final {SubsystemName}SysIdRoutine {subsystemName}SysId;

// In constructor
{subsystemName}SysId = new {SubsystemName}SysIdRoutine({subsystemName}Subsystem);

// In configureBindings() method
private void configureBindings() {
    // SysId commands - use triggers that won't interfere with normal operation
    // Example: Use secondary driver controller or specific button combinations
    
    secondaryController.a().onTrue({subsystemName}SysId.quasistaticForward());
    secondaryController.b().onTrue({subsystemName}SysId.quasistaticReverse());
    secondaryController.x().onTrue({subsystemName}SysId.dynamicForward());
    secondaryController.y().onTrue({subsystemName}SysId.dynamicReverse());
}
```

### Running SysId Tests

1. **Deploy Code**: Deploy robot code with SysId integration
2. **Connect SysId Tool**: Run `./gradlew SysId` to open SysId application
3. **Configure Project**: Create new project, set robot address (10.20.46.2)
4. **Run Tests in Order**:
   - Quasistatic Forward (slow ramp forward)
   - Quasistatic Reverse (slow ramp reverse)  
   - Dynamic Forward (quick step forward)
   - Dynamic Reverse (quick step reverse)
5. **Analyze Results**: SysId will calculate kS, kV, kA values

### Applying SysId Results

Update your constants class with the calculated values:

```java
public class {SubsystemName}Constants {
    // SysId-derived constants
    public static final class FeedforwardGains {
        public static final double KS = 0.25;  // Static friction (Volts)
        public static final double KV = 0.12;  // Velocity gain (V per rps)
        public static final double KA = 0.01;  // Acceleration gain (V per rps/s)
    }
    
    // Use these gains in motor configuration
    public static TalonFXConfiguration createMotorConfig() {
        var config = new TalonFXConfiguration();
        
        // Apply feedforward gains to slot 0
        config.Slot0.kS = FeedforwardGains.KS;
        config.Slot0.kV = FeedforwardGains.KV;
        config.Slot0.kA = FeedforwardGains.KA;
        
        // Start with conservative feedback gains, tune as needed
        config.Slot0.kP = 4.8;  // Adjust based on mechanism response
        config.Slot0.kI = 0.0;  // Usually not needed for position control
        config.Slot0.kD = 0.1;  // Helps reduce oscillation
        
        return config;
    }
}
```

### SysId Best Practices

#### Before Running Tests
- [ ] **Verify safe operating range** - ensure mechanism won't hit limits
- [ ] **Check conversion factors** - position/velocity units must be correct
- [ ] **Clear area** - ensure robot has space to move safely
- [ ] **Battery voltage** - use fresh battery (>12V) for consistent results

#### Test Execution Tips
- **Run multiple tests** - average results from 2-3 runs
- **Check data quality** - look for smooth ramps and consistent measurements
- **Start conservative** - use lower voltages initially, increase if needed
- **Watch for saturation** - if velocity plateaus early, reduce max voltage

#### Common Issues and Solutions

**Poor data quality (noisy/inconsistent):**
- Check motor connections and CAN bus health
- Verify encoder is properly connected and configured
- Ensure mechanism moves freely without binding

**Mechanism hits limits during test:**
- Reduce maximum voltage in SysId config
- Ensure adequate test space
- Consider software limits in subsystem

**Unrealistic gains (very high kS or negative values):**
- Check unit conversions (motor rotations vs mechanism rotations)
- Verify motor inversion settings
- Check if mechanism has significant friction or load

### Integration with Motion Magic

Once you have good feedforward gains, configure Motion Magic:

```java
public static TalonFXConfiguration createMotionMagicConfig() {
    var config = createMotorConfig(); // Gets feedforward gains
    
    // Motion Magic parameters
    var motionMagic = config.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = 80;   // rps - adjust for mechanism
    motionMagic.MotionMagicAcceleration = 160;    // rps/s - adjust for mechanism
    motionMagic.MotionMagicJerk = 1600;          // rps/s² - requires Pro license
    
    return config;
}

// Usage in subsystem
private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

public void setPosition(double targetRotations) {
    motor.setControl(motionMagicRequest.withPosition(targetRotations));
}
```

### Documentation and Tuning Notes

Keep a tuning log for each subsystem:

```java
/**
 * TUNING LOG - {SubsystemName}
 * 
 * SysId Results (Date: YYYY-MM-DD):
 * - kS: 0.25V (static friction)
 * - kV: 0.12V per rps (velocity feedforward)  
 * - kA: 0.01V per rps/s (acceleration feedforward)
 * - R²: 0.98 (data quality - should be >0.95)
 * 
 * PID Tuning Notes:
 * - kP: Started at 4.8, stable with good tracking
 * - kI: 0.0 - not needed, position control is stable
 * - kD: 0.1 - helps reduce overshoot
 * 
 * Motion Magic Settings:
 * - Cruise Velocity: 80 rps (mechanism limit)
 * - Acceleration: 160 rps/s (smooth motion)
 * - Jerk: 1600 rps/s² (Pro license required)
 * 
 * Known Issues:
 * - None currently
 * 
 * Last Updated: YYYY-MM-DD by [Your Name]
 */
```

## Quick Start Checklist

- [ ] Create package directory: `src/main/java/org/tahomarobotics/robot/{subsystemname}/`
- [ ] Create `{SubsystemName}Constants.java` with hardware IDs and control constants
- [ ] Create `{SubsystemName}Subsystem.java` extending AbstractSubsystem
- [ ] Create `{SubsystemName}.java` command factory class
- [ ] Create `{SubsystemName}Simulation.java` extending AbstractSimulation  
- [ ] **Create `{SubsystemName}SysIdRoutine.java` for motor characterization**
- [ ] **Add SysId methods to subsystem (setVoltage, getPosition, getVelocity, etc.)**
- [ ] **Add SysId commands to RobotContainer**
- [ ] **Run SysId tests and update constants with results**
- [ ] Add subsystem to RobotContainer constructor and imports
- [ ] Update RobotMap with CAN IDs (if needed)
- [ ] Create basic unit test
- [ ] **Document tuning results and settings**
- [ ] Test in simulation: `./gradlew simulateJava`
- [ ] Verify logging in AdvantageScope

## Common Patterns to Follow

1. **Use MIT License Header**: Copy from existing files
2. **Follow Naming Conventions**: PascalCase for classes, camelCase for instances
3. **Extend AbstractSubsystem**: Use the project's base subsystem class
4. **Use RobustConfigurator**: For reliable motor configuration
5. **Log with AdvantageKit**: Use Logger.recordOutput() in periodic()
6. **Optimize CAN Usage**: Set appropriate status signal frequencies
7. **Reuse Control Requests**: Create static instances in Constants class
8. **Include Simulation**: Always create a simulation class for testing
9. **Add Resource Cleanup**: Implement AutoCloseable and close() methods
10. **Use CommandLogger**: Log command starts/ends for debugging

This structure provides a solid foundation for any new subsystem while maintaining consistency with the existing codebase architecture.