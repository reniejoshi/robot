# MapleSim API Documentation

## Overview
MapleSim is a comprehensive physics simulation framework specifically designed for FRC robotics. Developed by Shenzhen Robotics Alliance, it provides high-fidelity physics simulation capabilities with seamless integration into WPILib and FRC development workflows.

## Version Information
- **Library Version**: 0.3.11  
- **Group ID**: org.ironmaple
- **Artifact**: maplesim-java
- **Maven URL**: https://shenzhen-robotics-alliance.github.io/maple-sim/vendordep/repos/releases
- **Physics Engine**: Dyn4j 5.0.2 (included dependency)

## Core Components

### Simulation Environment
MapleSim provides a realistic physics environment that simulates FRC field conditions, robot dynamics, and game piece interactions.

```java
import org.ironmaple.simulation.SimulationWorld;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class RobotSimulation {
    private final SimulationWorld world;
    private final SwerveDriveSimulation driveSimulation;
    
    public RobotSimulation() {
        // Create simulation world with FRC field
        world = new SimulationWorld();
        
        // Initialize drive simulation
        driveSimulation = new SwerveDriveSimulation(world, chassisConfig);
    }
    
    public void updateSimulation() {
        world.simulationSubTick(0.020); // 20ms physics step
    }
}
```

### Drive System Simulation
MapleSim provides realistic drivetrain simulation with proper physics modeling.

```java
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

// Configure chassis parameters
DriveTrainSimulationConfig chassisConfig = new DriveTrainSimulationConfig()
    .withBumperWidthX(0.9)  // 36 inches
    .withBumperWidthY(0.9)  // 36 inches
    .withRobotMass(60.0)    // 60 kg robot mass
    .withMOI(6.0);          // Moment of inertia

// Swerve module configuration
SwerveModuleSimulationConfig moduleConfig = new SwerveModuleSimulationConfig()
    .withWheelRadius(0.0508)  // 2 inch wheels
    .withDriveGearRatio(6.75) // MK4i L2 gearing
    .withSteerGearRatio(150.0/7.0)
    .withDriveMotor(DCMotor.getKrakenX60Foc(1))
    .withSteerMotor(DCMotor.getNeo550(1));
```

### Field Interaction Simulation
```java
import org.ironmaple.simulation.gamepieces.GamePieceSimulation;

public class FieldSimulation {
    private final GamePieceSimulation gamePieceSimulation;
    
    public FieldSimulation() {
        // Initialize game piece physics
        gamePieceSimulation = new GamePieceSimulation(world);
        
        // Add game pieces to field
        addGamePiecesToField();
    }
    
    private void addGamePiecesToField() {
        // Add notes/game pieces at field positions
        gamePieceSimulation.addGamePiece(
            GamePieceType.NOTE, 
            new Pose2d(2.0, 5.5, new Rotation2d())
        );
    }
}
```

## Integration with Robot Code

### Subsystem Integration
MapleSim integrates with existing subsystems through simulation interfaces:

```java
public class ChassisSimulation {
    private final SwerveDriveSimulation driveSimulation;
    private final SwerveModule[] moduleSimulations;
    
    public ChassisSimulation(SimulationWorld world) {
        // Create drive simulation
        driveSimulation = new SwerveDriveSimulation(world, chassisConfig);
        
        // Initialize module simulations
        moduleSimulations = new SwerveModule[4];
        for (int i = 0; i < 4; i++) {
            moduleSimulations[i] = new SwerveModuleSimulation(moduleConfigs[i]);
        }
    }
    
    public void updateInputs(ChassisIOInputsAutoLogged inputs) {
        // Get simulated sensor readings
        inputs.gyroYawPosition = driveSimulation.getGyroReading();
        inputs.odometryPose = driveSimulation.getSimulatedDriveTrainPose();
        
        // Update module inputs
        for (int i = 0; i < 4; i++) {
            inputs.moduleStates[i] = moduleSimulations[i].getCurrentState();
        }
    }
    
    public void setDesiredStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            moduleSimulations[i].setDesiredState(states[i]);
        }
    }
}
```

### Vision Simulation Integration
```java
import org.ironmaple.simulation.vision.VisionSystemSimulation;

public class VisionSimulation {
    private final VisionSystemSimulation visionSim;
    
    public VisionSimulation(SimulationWorld world) {
        // Create vision system simulation
        visionSim = new VisionSystemSimulation(world);
        
        // Configure cameras
        addCamerasToSimulation();
    }
    
    private void addCamerasToSimulation() {
        // Add AprilTag camera
        CameraSimulationConfig aprilTagCameraConfig = new CameraSimulationConfig()
            .withCameraName("apriltag-camera")
            .withRobotToCameraTransform(robotToCameraTransform)
            .withCameraResolution(1280, 720)
            .withFOV(Math.toRadians(70))
            .withLatency(0.035); // 35ms latency
            
        visionSim.addCamera(aprilTagCameraConfig);
    }
    
    public void updateSimulation(Pose2d robotPose) {
        visionSim.updateVisionEstimation(robotPose);
    }
}
```

## Physics Configuration

### Robot Physical Properties
```java
// Mass and inertia properties
DriveTrainSimulationConfig config = new DriveTrainSimulationConfig()
    .withRobotMass(60.0)        // Total robot mass (kg)
    .withMOI(6.0)               // Rotational inertia (kg⋅m²)
    .withBumperWidthX(0.889)    // Robot width with bumpers (m)
    .withBumperWidthY(0.889)    // Robot length with bumpers (m)
    .withTrackWidthX(0.521)     // Distance between left/right wheels (m)
    .withTrackWidthY(0.521);    // Distance between front/back wheels (m)
```

### Motor and Gearing Configuration
```java
// Swerve module configuration
SwerveModuleSimulationConfig moduleConfig = new SwerveModuleSimulationConfig()
    // Physical properties
    .withWheelRadius(0.0508)        // 2" wheels in meters
    .withWheelCOF(1.2)              // Coefficient of friction
    
    // Gearing
    .withDriveGearRatio(6.75)       // Drive reduction (MK4i L2)
    .withSteerGearRatio(150.0/7.0)  // Steering reduction
    
    // Motors
    .withDriveMotor(DCMotor.getKrakenX60Foc(1))  // Kraken X60 drive
    .withSteerMotor(DCMotor.getNeo550(1))        // NEO 550 steer
    
    // Current limits
    .withDriveCurrentLimit(80.0)    // 80A drive current limit
    .withSteerCurrentLimit(20.0);   // 20A steer current limit
```

### Field Setup
```java
public class Arena2025Reefscape {
    private final SimulationWorld world;
    
    public Arena2025Reefscape() {
        world = new SimulationWorld();
        
        // Add field boundaries
        addFieldBoundaries();
        
        // Add field elements
        addFieldElements();
        
        // Add game pieces
        addGamePieces();
    }
    
    private void addFieldBoundaries() {
        // Create field perimeter walls
        world.addFieldBoundary(0, 0, 16.54, 8.21); // FRC field dimensions
    }
    
    private void addFieldElements() {
        // Add scoring areas, barriers, etc.
        world.addStaticObstacle(/* reef structure coordinates */);
    }
}
```

## Usage Patterns in This Codebase

Based on the import analysis, MapleSim is used for:

### Complete Robot Simulation
- **Arena2025Reefscape**: Full FRC 2025 field simulation
- **ChassisSimulation**: Swerve drive physics simulation  
- **VisionSimulation**: Camera and AprilTag simulation
- **GamePieceSimulation**: Note/coral interaction physics
- **HumanPlayerSimulation**: Human player station simulation

### Typical Usage Pattern
```java
public class Robot extends LoggedRobot {
    private SimulationWorld simulationWorld;
    private Arena2025Reefscape fieldSimulation;
    
    @Override
    public void robotInit() {
        if (Robot.isSimulation()) {
            // Initialize MapleSim components
            simulationWorld = new SimulationWorld();
            fieldSimulation = new Arena2025Reefscape(simulationWorld);
        }
    }
    
    @Override
    public void simulationPeriodic() {
        if (simulationWorld != null) {
            // Update physics simulation
            simulationWorld.simulationSubTick(0.020);
            
            // Update robot pose in field
            fieldSimulation.updateRobotPose(robotContainer.getChassis().getPose());
        }
    }
}
```

## Advanced Features

### Collision Detection
```java
public class CollisionSimulation {
    public void checkCollisions(Pose2d robotPose) {
        // Check robot-to-wall collisions
        if (world.isColliding(robotPose, robotBoundary)) {
            handleWallCollision();
        }
        
        // Check robot-to-game-piece collisions
        List<GamePiece> collidingPieces = world.getCollidingGamePieces(robotPose);
        for (GamePiece piece : collidingPieces) {
            handleGamePieceCollision(piece);
        }
    }
}
```

### Performance Optimization
```java
public class SimulationManager {
    private int simulationCounter = 0;
    
    public void updateSimulation() {
        // Run physics at full rate
        world.simulationSubTick(0.020);
        
        // Update heavy computations less frequently
        simulationCounter++;
        if (simulationCounter % 5 == 0) { // Every 100ms
            updateExpensiveSimulations();
        }
    }
    
    private void updateExpensiveSimulations() {
        // Update vision simulation
        visionSimulation.update();
        
        // Update field visualization
        updateFieldVisualization();
    }
}
```

### Integration with AdvantageKit
```java
public class SimulationInputs {
    @AutoLog
    public static class SimulationIOInputs {
        public Pose2d simulatedPose = new Pose2d();
        public double[] gamePickupPositions = new double[0];
        public boolean[] gamePieceCollected = new boolean[0];
        public double simulationTimestamp = 0.0;
    }
    
    public void updateInputs(SimulationIOInputsAutoLogged inputs) {
        inputs.simulatedPose = driveSimulation.getSimulatedDriveTrainPose();
        inputs.simulationTimestamp = Timer.getFPGATimestamp();
        
        // Log game piece states
        List<GamePiece> pieces = world.getGamePieces();
        inputs.gamePickupPositions = pieces.stream()
            .mapToDouble(p -> p.getPosition().getX())
            .toArray();
    }
}
```

## Field Coordinate System

MapleSim uses the standard FRC coordinate system:
- **Origin**: Blue alliance driver station wall
- **X-axis**: Points toward red alliance (field length)
- **Y-axis**: Points toward center of field (field width)  
- **Rotation**: Counter-clockwise positive
- **Units**: Meters for distance, radians for angles

## Simulation Accuracy

### Physics Fidelity
- **Real-time physics**: Accurate collision detection and response
- **Motor modeling**: Includes realistic torque curves and current limits
- **Friction modeling**: Wheel slip and traction characteristics
- **Inertial effects**: Proper mass and rotational inertia simulation

### Performance Considerations
```java
// Optimize simulation performance
SimulationConfig config = new SimulationConfig()
    .withPhysicsTimeStep(0.005)     // 5ms physics timestep
    .withRenderingFPS(60)           // 60 FPS visualization
    .withCollisionPrecision(HIGH)   // High-precision collisions
    .withMultithreading(true);      // Enable parallel processing
```

## Best Practices

1. **Initialization**: Set up simulation components in robotInit() for simulation builds only
2. **Performance**: Update heavy simulations at reduced frequencies
3. **Accuracy**: Use realistic physical parameters (mass, inertia, friction)
4. **Integration**: Ensure simulation inputs match real hardware behavior
5. **Testing**: Use simulation to test autonomous routines and edge cases
6. **Visualization**: Leverage Field2d integration for debugging

## Troubleshooting

1. **Poor Performance**: Reduce physics timestep or collision precision
2. **Unrealistic Behavior**: Check physical parameters (mass, inertia, friction)
3. **Integration Issues**: Ensure simulation updates happen in simulationPeriodic()
4. **Collision Problems**: Verify robot and field geometry setup
5. **Sensor Mismatch**: Confirm simulated sensor outputs match real hardware

## Additional Resources
- [MapleSim Documentation](https://shenzhen-robotics-alliance.github.io/maple-sim/)
- [Dyn4j Physics Engine](https://dyn4j.org/)
- [FRC Simulation Guide](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/)
- [Physics Simulation Best Practices](https://shenzhen-robotics-alliance.github.io/maple-sim/best-practices/)