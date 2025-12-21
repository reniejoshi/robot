# WPILib API Documentation

## Overview
WPILib (WPI Robotics Library) is the core software framework for FRC (FIRST Robotics Competition) robots. It provides the fundamental building blocks for robot control, including hardware interfaces, control algorithms, mathematical utilities, and simulation capabilities.

## Version Information  
- **Library Version**: 2025.3.2 (FRC 2025 season)
- **Group ID**: edu.wpi.first
- **Core Modules**:
  - wpilibj (Java API)
  - wpimath (Mathematical utilities)
  - wpiutil (Utility functions)
  - hal (Hardware Abstraction Layer)
  - cscore (Camera/Vision utilities)

## Core Framework Components

### Robot Base Classes
```java
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    @Override
    public void robotInit() {
        // Initialize robot hardware and subsystems
    }
    
    @Override
    public void robotPeriodic() {
        // Runs every 20ms regardless of mode
    }
    
    @Override
    public void autonomousInit() {
        // Initialize autonomous mode
    }
    
    @Override
    public void autonomousPeriodic() {
        // Runs every 20ms during autonomous
    }
    
    @Override
    public void teleopInit() {
        // Initialize teleop mode
    }
    
    @Override
    public void teleopPeriodic() {
        // Runs every 20ms during teleop
    }
}
```

### Command-Based Framework
```java
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveSubsystem extends SubsystemBase {
    @Override
    public void periodic() {
        // Called every 20ms
    }
    
    public Command driveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
        return run(() -> {
            drive(forward.getAsDouble(), rotation.getAsDouble());
        });
    }
}
```

## Hardware Interfaces

### Motor Controllers
```java
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.PWMTalonSRX;

// PWM motor controllers
PWMVictorSPX leftMotor = new PWMVictorSPX(0);
PWMTalonSRX rightMotor = new PWMTalonSRX(1);

// Basic motor control
leftMotor.set(0.5);  // 50% forward
rightMotor.set(-0.5); // 50% reverse
```

### Sensors
```java
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

// Quadrature encoder
Encoder encoder = new Encoder(0, 1); // DIO ports 0 and 1
encoder.setDistancePerPulse(0.1); // 0.1 meters per pulse

// Gyroscope
ADXRS450_Gyro gyro = new ADXRS450_Gyro();
double angle = gyro.getAngle();

// Analog sensor
AnalogInput potentiometer = new AnalogInput(0);
double voltage = potentiometer.getVoltage();

// Digital sensor
DigitalInput limitSwitch = new DigitalInput(5);
boolean isPressed = !limitSwitch.get(); // Inverted for normally open switch
```

### Driver Station Integration
```java
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;

XboxController controller = new XboxController(0);

// Driver Station information
boolean isEnabled = DriverStation.isEnabled();
boolean isAutonomous = DriverStation.isAutonomous();
boolean isFMSAttached = DriverStation.isFMSAttached();
DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

// Controller input
double leftY = controller.getLeftY();
double rightX = controller.getRightX();
boolean aButton = controller.getAButton();
```

## Mathematical Utilities (WPIMath)

### Geometry Classes
```java
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;

// 2D Pose (position + rotation)
Pose2d robotPose = new Pose2d(1.5, 2.0, Rotation2d.fromDegrees(45));
Translation2d position = robotPose.getTranslation();
Rotation2d rotation = robotPose.getRotation();

// Transforms
Transform2d transform = new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(90));
Pose2d newPose = robotPose.plus(transform);

// 3D geometry for vision
Pose3d robotPose3d = new Pose3d(1.5, 2.0, 0.0, new Rotation3d(0, 0, Math.PI/4));
Transform3d cameraTransform = new Transform3d(
    new Translation3d(0.3, 0, 0.2), // 30cm forward, 20cm up
    new Rotation3d(0, -Math.PI/12, 0) // 15 degrees down
);
```

### Control Theory
```java
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.filter.SlewRateLimiter;

// PID Controller
PIDController pidController = new PIDController(1.0, 0.0, 0.1);
pidController.setTolerance(0.05); // 5cm tolerance
double output = pidController.calculate(currentPosition, targetPosition);

// Slew rate limiter (acceleration limiting)
SlewRateLimiter rateLimiter = new SlewRateLimiter(3.0); // 3 units/second max change
double limitedOutput = rateLimiter.calculate(desiredOutput);

// Profiled PID (with motion profiling)
TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2.0, 1.0);
ProfiledPIDController profiledPID = new ProfiledPIDController(1.0, 0.0, 0.1, constraints);
```

### Kinematics and Odometry
```java
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.odometry.*;

// Differential drive kinematics
DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.6); // 60cm track width
ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0.0, 1.0); // 2 m/s forward, 1 rad/s rotation
DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

// Swerve drive kinematics
Translation2d[] modulePositions = {
    new Translation2d(0.3, 0.3),   // Front left
    new Translation2d(0.3, -0.3),  // Front right  
    new Translation2d(-0.3, 0.3),  // Back left
    new Translation2d(-0.3, -0.3)  // Back right
};
SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(modulePositions);

// Odometry
DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    gyro.getRotation2d(), 
    leftEncoder.getDistance(), 
    rightEncoder.getDistance(),
    new Pose2d() // Starting pose
);
```

### State Space Control
```java
import edu.wpi.first.math.system.*;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;

// Linear system identification
LinearSystem<N2, N2, N2> elevatorSystem = LinearSystemId.createElevatorSystem(
    DCMotor.getNEO(2), // 2 NEO motors
    70.0, // 70 kg mass
    0.05, // 5cm radius
    4.0   // 4:1 gearing
);

// LQR controller
LinearQuadraticRegulator<N2, N2, N2> lqr = new LinearQuadraticRegulator<>(
    elevatorSystem,
    VecBuilder.fill(0.02, 0.4), // State tolerances [position, velocity]
    VecBuilder.fill(12.0, 12.0), // Input costs [voltage]
    0.02 // Control loop period
);
```

## Simulation Framework

### Physics Simulation
```java
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.math.system.plant.DCMotor;

// Differential drive simulation
DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(
    DrivetrainIdentification.createDrivetrainVelocitySystem(
        DCMotor.getNEO(2), // 2 NEOs per side
        60.0,  // Robot mass (kg)
        0.0762, // Wheel radius (m)
        0.7112, // Track width (m)
        Arrays.asList(1.0/8.45, 1.0/8.45), // Gearing
        7.5  // MOI
    ),
    DCMotor.getNEO(2),
    8.45, // Gearing
    0.0762, // Wheel radius
    0.7112  // Track width
);

// Elevator simulation
ElevatorSim elevatorSim = new ElevatorSim(
    LinearSystemId.createElevatorSystem(
        DCMotor.getNEO(2), 
        70, // Carriage mass
        0.05, // Drum radius
        4.0 // Gearing
    ),
    DCMotor.getNEO(2),
    0.0, // Min height
    3.0, // Max height
    true, // Simulate gravity
    0.0  // Starting height
);
```

### Field2d Visualization
```java
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
    private final Field2d field = new Field2d();
    
    public DriveSubsystem() {
        SmartDashboard.putData("Field", field);
    }
    
    @Override
    public void periodic() {
        // Update robot pose on field
        field.setRobotPose(getPose());
        
        // Show trajectory
        field.getObject("trajectory").setTrajectory(currentTrajectory);
        
        // Show vision targets
        field.getObject("vision").setPoses(visionTargets);
    }
}
```

## Network Communication

### NetworkTables
```java
import edu.wpi.first.networktables.*;

// Get table instance
NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");

// Publish data
table.getEntry("Robot X").setDouble(robotPose.getX());
table.getEntry("Robot Y").setDouble(robotPose.getY());
table.getEntry("Auto Selected").setString(selectedAuto);

// Subscribe to data
NetworkTableEntry visionHasTarget = table.getEntry("Vision Has Target");
boolean hasTarget = visionHasTarget.getBoolean(false);
```

### SmartDashboard
```java
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

// Simple data publishing
SmartDashboard.putNumber("Drive/Left Velocity", leftVelocity);
SmartDashboard.putBoolean("Intake/Has Game Piece", hasGamePiece);
SmartDashboard.putString("Auto/Current Command", currentCommand.getName());

// Autonomous chooser
SendableChooser<Command> autoChooser = new SendableChooser<>();
autoChooser.setDefaultOption("Drive Forward", driveForwardAuto);
autoChooser.addOption("Score and Leave", scoreAndLeaveAuto);
SmartDashboard.putData("Auto Chooser", autoChooser);
```

## Usage Patterns in This Codebase

Based on the import analysis, WPILib is extensively used throughout:

### Core Robot Framework
```java
// Robot extends LoggedRobot (which extends TimedRobot)
public class Robot extends LoggedRobot {
    // Standard WPILib robot structure with AdvantageKit logging
}
```

### Subsystem Architecture
- **All subsystems** extend SubsystemBase
- **Command-based** framework for robot actions
- **Periodic methods** for continuous updates
- **Hardware abstraction** through IO interfaces

### Mathematical Operations
- **Pose2d/Pose3d** for robot localization
- **Transform2d/Transform3d** for coordinate transformations  
- **Rotation2d** for angle representations
- **ChassisSpeeds** for velocity commands
- **PID controllers** for closed-loop control

### Simulation Integration
- **Field2d** for robot visualization
- **Physics simulation** for testing
- **NetworkTables** for telemetry

## Best Practices for FRC

### Resource Management
```java
@Override
public void close() {
    // Clean up hardware resources
    motor.close();
    encoder.close();
}
```

### Safety and Validation
```java
public void setSpeeds(double left, double right) {
    // Validate inputs
    left = MathUtil.clamp(left, -1.0, 1.0);
    right = MathUtil.clamp(right, -1.0, 1.0);
    
    // Apply safety limits
    if (DriverStation.isDisabled()) {
        left = right = 0.0;
    }
    
    leftMotor.set(left);
    rightMotor.set(right);
}
```

### Performance Optimization
```java
@Override
public void periodic() {
    // Update odometry every loop
    odometry.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());
    
    // Expensive operations less frequently
    if (loopCounter % 5 == 0) { // Every 100ms
        updateVisionPose();
    }
    
    loopCounter++;
}
```

### Competition Readiness
```java
public void configureForCompetition() {
    // Reduce NetworkTables traffic
    if (DriverStation.isFMSAttached()) {
        // Minimize telemetry
        table.getEntry("Debug Data").unpublish();
    }
    
    // Set appropriate timeouts
    motor.configAllSettings(config, 50); // 50ms timeout
}
```

## Integration with Third-Party Libraries

### Phoenix 6 Integration
```java
// WPILib provides base interfaces that Phoenix 6 implements
MotorController phoenixMotor = new TalonFX(1);
DifferentialDrive drive = new DifferentialDrive(phoenixMotor, phoenixMotor);
```

### Vision Integration  
```java
// WPILib geometry classes work seamlessly with PhotonVision
Pose2d visionPose = visionResult.getBestTarget().getBestCameraToTarget().toPose2d();
odometry.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
```

## Advanced Features

### Event Loop
```java
import edu.wpi.first.wpilibj.event.EventLoop;

EventLoop eventLoop = new EventLoop();
BooleanEvent hasGamePiece = new BooleanEvent(eventLoop, this::hasGamePiece);

// Trigger actions on events
hasGamePiece.onTrue(Commands.runOnce(() -> rumbleController()));
hasGamePiece.onFalse(Commands.runOnce(() -> stopRumble()));
```

### DataLog Integration
```java
import edu.wpi.first.util.datalog.*;

DataLog log = DataLogManager.getLog();
DoubleLogEntry velocityLog = new DoubleLogEntry(log, "/drive/velocity");
BooleanLogEntry enabledLog = new BooleanLogEntry(log, "/robot/enabled");

// Log data
velocityLog.append(getCurrentVelocity());
enabledLog.append(DriverStation.isEnabled());
```

## Additional Resources
- [WPILib Documentation](https://docs.wpilib.org/)
- [WPILib GitHub Repository](https://github.com/wpilibsuite/allwpilib)
- [FRC Programming Best Practices](https://docs.wpilib.org/en/stable/docs/software/best-practices/)
- [Command-Based Programming Guide](https://docs.wpilib.org/en/stable/docs/software/commandbased/)