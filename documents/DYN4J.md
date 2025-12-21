# Dyn4j Physics Engine Documentation

## Overview
Dyn4j is a 2D collision detection and physics engine for Java, designed for real-time applications. It provides comprehensive physics simulation capabilities including rigid body dynamics, collision detection, constraint solving, and continuous collision detection. In this FRC robot project, Dyn4j is used as the underlying physics engine for MapleSim.

## Version Information
- **Library Version**: 5.0.2
- **Group ID**: org.dyn4j  
- **Artifact**: dyn4j
- **Included Via**: MapleSim dependency
- **Official Website**: https://dyn4j.org/

## Core Components

### World Simulation
Dyn4j provides the foundational physics world that MapleSim uses for robot simulation:

```java
import org.dyn4j.dynamics.World;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.*;

public class PhysicsWorld {
    private final World world;
    
    public PhysicsWorld() {
        // Create physics world with gravity
        world = new World();
        world.setGravity(new Vector2(0, -9.81)); // Standard gravity
        
        // Configure world settings
        world.getSettings().setStepFrequency(1.0 / 50.0); // 50 Hz physics
        world.getSettings().setMaximumTranslation(4.0);   // Max movement per step
    }
    
    public void step(double deltaTime) {
        world.step(1, deltaTime);
    }
}
```

### Rigid Body Creation
```java
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;

public class RobotBody {
    public static Body createRobotChassis(double width, double length, double mass) {
        // Create rectangular chassis geometry
        Rectangle chassisShape = Geometry.createRectangle(width, length);
        
        // Create rigid body
        Body chassis = new Body();
        chassis.addFixture(chassisShape);
        chassis.setMass(MassType.NORMAL); // Calculate mass automatically
        chassis.getMass().setMass(mass);
        
        // Set physical properties
        chassis.setLinearDamping(0.1);    // Air resistance
        chassis.setAngularDamping(0.05);  // Rotational damping
        
        return chassis;
    }
    
    public static Body createWheel(double radius, double mass) {
        // Create circular wheel geometry
        Circle wheelShape = Geometry.createCircle(radius);
        
        Body wheel = new Body();
        wheel.addFixture(wheelShape);
        wheel.setMass(MassType.NORMAL);
        wheel.getMass().setMass(mass);
        
        // High friction for traction
        wheel.getFixture(0).setFriction(1.2);
        wheel.getFixture(0).setRestitution(0.3); // Bounce factor
        
        return wheel;
    }
}
```

### Collision Detection
```java
import org.dyn4j.collision.*;
import org.dyn4j.dynamics.contact.Contact;

public class CollisionHandler {
    public void setupCollisionDetection(World world) {
        // Set up collision listener
        world.addContactListener(new ContactListener() {
            @Override
            public boolean begin(ContactCollisionData<Body, BodyFixture> collision) {
                Body bodyA = collision.getBody1();
                Body bodyB = collision.getBody2();
                
                // Handle robot-to-wall collision
                if (isRobot(bodyA) && isWall(bodyB)) {
                    handleWallCollision(bodyA, bodyB);
                }
                
                // Handle robot-to-game-piece collision
                if (isRobot(bodyA) && isGamePiece(bodyB)) {
                    handleGamePieceCollision(bodyA, bodyB);
                }
                
                return true; // Allow collision to proceed
            }
        });
    }
}
```

### Constraint Systems
```java
import org.dyn4j.dynamics.joint.*;

public class ConstraintSetup {
    public void createSwerveModule(World world, Body chassis, Body wheel) {
        // Create revolute joint for wheel rotation
        RevoluteJoint<Body> wheelJoint = new RevoluteJoint<>(
            chassis, wheel, wheel.getWorldCenter()
        );
        
        // Enable motor for wheel drive
        wheelJoint.setMotorEnabled(true);
        wheelJoint.setMaximumMotorTorque(100.0); // Nm
        wheelJoint.setMotorSpeed(0.0); // Initial speed
        
        world.addJoint(wheelJoint);
        
        // Create steering constraint (could be another revolute joint)
        // for swerve module steering mechanism
    }
}
```

## Integration with MapleSim

### Physics World Wrapper
MapleSim wraps Dyn4j's World class to provide FRC-specific functionality:

```java
// MapleSim's SimulationWorld likely extends or wraps Dyn4j's World
public class SimulationWorld {
    private final World dyn4jWorld;
    private final List<Body> robotBodies;
    private final List<Body> gameObjects;
    
    public SimulationWorld() {
        dyn4jWorld = new World();
        configurePhysicsWorld();
    }
    
    private void configurePhysicsWorld() {
        // FRC field has no gravity (horizontal simulation)
        dyn4jWorld.setGravity(World.ZERO_GRAVITY);
        
        // Configure for real-time performance
        dyn4jWorld.getSettings().setStepFrequency(1.0 / 50.0); // 50 Hz
        dyn4jWorld.getSettings().setContinuousDetectionMode(
            ContinuousDetectionMode.BULLETS_ONLY
        );
    }
    
    public void simulationSubTick(double deltaTime) {
        dyn4jWorld.step(1, deltaTime);
    }
}
```

### Robot Physics Integration
```java
public class SwerveDrivePhysics {
    private final Body chassisBody;
    private final Body[] wheelBodies;
    private final RevoluteJoint<Body>[] wheelJoints;
    
    public SwerveDrivePhysics(World world, DriveConfig config) {
        // Create chassis body
        chassisBody = createChassisBody(config);
        world.addBody(chassisBody);
        
        // Create wheel bodies and joints
        wheelBodies = new Body[4];
        wheelJoints = new RevoluteJoint[4];
        
        for (int i = 0; i < 4; i++) {
            wheelBodies[i] = createWheelBody(config.wheelRadius, config.wheelMass);
            wheelJoints[i] = createWheelJoint(chassisBody, wheelBodies[i], config.modulePositions[i]);
            
            world.addBody(wheelBodies[i]);
            world.addJoint(wheelJoints[i]);
        }
    }
    
    public void setWheelSpeeds(double[] speeds) {
        for (int i = 0; i < 4; i++) {
            wheelJoints[i].setMotorSpeed(speeds[i]);
        }
    }
    
    public Pose2d getRobotPose() {
        Vector2 position = chassisBody.getWorldCenter();
        double rotation = chassisBody.getTransform().getRotationAngle();
        
        return new Pose2d(
            position.x, position.y, 
            new Rotation2d(rotation)
        );
    }
}
```

## Physics Properties and Tuning

### Material Properties
```java
// Configure realistic material properties
public class MaterialProperties {
    public static void setupRobotMaterials(Body robotBody) {
        BodyFixture fixture = robotBody.getFixture(0);
        
        // Aluminum chassis properties
        fixture.setDensity(2.7); // kg/m² for 2D simulation
        fixture.setFriction(0.6); // Metal-on-carpet friction
        fixture.setRestitution(0.1); // Low bounce
    }
    
    public static void setupWheelMaterials(Body wheelBody) {
        BodyFixture fixture = wheelBody.getFixture(0);
        
        // Rubber wheel properties
        fixture.setFriction(1.2); // High traction
        fixture.setRestitution(0.3); // Moderate bounce
    }
    
    public static void setupGamePieceMaterials(Body gamePiece) {
        BodyFixture fixture = gamePiece.getFixture(0);
        
        // Foam/plastic game piece properties
        fixture.setFriction(0.4); // Low friction
        fixture.setRestitution(0.8); // High bounce
    }
}
```

### Performance Optimization
```java
public class PhysicsOptimization {
    public static void optimizeForRealTime(World world) {
        WorldSettings settings = world.getSettings();
        
        // Balance accuracy vs. performance
        settings.setStepFrequency(1.0 / 50.0); // 50 Hz physics
        settings.setVelocityConstraintSolverIterations(8); // Reduced iterations
        settings.setPositionConstraintSolverIterations(3);
        
        // Continuous collision detection settings
        settings.setContinuousDetectionMode(ContinuousDetectionMode.BULLETS_ONLY);
        settings.setMaximumTranslation(2.0); // Limit max movement per step
        settings.setMaximumRotation(Math.PI / 4); // Limit max rotation per step
        
        // Performance tuning
        settings.setBaumgarte(0.2); // Constraint violation correction
        settings.setLinearTolerance(0.005); // Position tolerance
        settings.setAngularTolerance(Math.toRadians(2)); // Rotation tolerance
    }
}
```

## Common Physics Patterns

### Force Application
```java
public class ForceApplication {
    public static void applyMotorForces(Body wheelBody, double torque) {
        // Apply torque directly to wheel
        wheelBody.applyTorque(torque);
    }
    
    public static void applyDriveForce(Body chassisBody, Vector2 force) {
        // Apply linear force to chassis center of mass
        chassisBody.applyForce(force);
    }
    
    public static void applyFrictionForce(Body body, double frictionCoeff) {
        Vector2 velocity = body.getLinearVelocity();
        if (velocity.getMagnitudeSquared() > 0) {
            Vector2 friction = velocity.copy().normalize().multiply(-frictionCoeff);
            body.applyForce(friction);
        }
    }
}
```

### Collision Response
```java
public class CollisionResponse {
    public static void handleWallCollision(Body robot, Contact contact) {
        // Get collision normal
        Vector2 normal = contact.getNormal();
        
        // Apply impulse to separate bodies
        double separation = contact.getDepth();
        Vector2 impulse = normal.copy().multiply(separation * robot.getMass().getMass());
        robot.applyImpulse(impulse);
        
        // Reduce velocity along collision normal
        Vector2 velocity = robot.getLinearVelocity();
        double normalVelocity = velocity.dot(normal);
        if (normalVelocity < 0) {
            Vector2 correction = normal.copy().multiply(-normalVelocity * 0.8);
            robot.setLinearVelocity(velocity.add(correction));
        }
    }
}
```

### Game Piece Interaction
```java
public class GamePiecePhysics {
    public static void simulateIntake(Body robot, Body gamePiece, double intakeForce) {
        // Calculate direction from game piece to robot
        Vector2 robotPos = robot.getWorldCenter();
        Vector2 piecePos = gamePiece.getWorldCenter();
        Vector2 direction = robotPos.subtract(piecePos).normalize();
        
        // Apply attraction force to game piece
        Vector2 force = direction.multiply(intakeForce);
        gamePiece.applyForce(force);
        
        // Check if game piece is close enough to "collect"
        double distance = robotPos.distance(piecePos);
        if (distance < COLLECTION_THRESHOLD) {
            // Remove from physics world (collected)
            // This would be handled by MapleSim's game piece management
        }
    }
}
```

## Performance Considerations

### Simulation Frequency
```java
public class SimulationTiming {
    private static final double PHYSICS_TIMESTEP = 1.0 / 50.0; // 50 Hz
    private static final double RENDER_TIMESTEP = 1.0 / 60.0;  // 60 FPS
    
    public void updatePhysics() {
        // Physics runs at fixed timestep for stability
        world.step(1, PHYSICS_TIMESTEP);
    }
    
    public void interpolateForRendering(double alpha) {
        // Interpolate positions for smooth visual rendering
        // between physics steps
        for (Body body : bodies) {
            Vector2 currentPos = body.getWorldCenter();
            Vector2 previousPos = getPreviousPosition(body);
            Vector2 renderPos = previousPos.add(currentPos.subtract(previousPos).multiply(alpha));
            updateVisualPosition(body, renderPos);
        }
    }
}
```

### Memory Management
```java
public class PhysicsMemoryManagement {
    private final ObjectPool<Vector2> vectorPool = new ObjectPool<>();
    private final ObjectPool<Body> bodyPool = new ObjectPool<>();
    
    public Vector2 getVector() {
        return vectorPool.get(() -> new Vector2());
    }
    
    public void releaseVector(Vector2 vector) {
        vector.set(0, 0); // Reset
        vectorPool.release(vector);
    }
    
    public void cleanupPhysicsObjects() {
        // Remove inactive bodies to prevent memory leaks
        world.removeAllBodiesInactive(true);
        world.removeAllJointsInactive(true);
    }
}
```

## Integration Points with FRC Code

### Sensor Simulation
```java
// Dyn4j physics provides realistic sensor readings
public class SimulatedSensors {
    public static double getGyroReading(Body robotBody) {
        return Math.toDegrees(robotBody.getTransform().getRotationAngle());
    }
    
    public static double getEncoderReading(Body wheelBody, double wheelRadius) {
        double angularVelocity = wheelBody.getAngularVelocity();
        return angularVelocity * wheelRadius; // Linear velocity
    }
    
    public static boolean getLimitSwitchReading(Body robot, Body obstacle) {
        // Check if robot is in contact with obstacle
        return world.isInContact(robot, obstacle);
    }
}
```

### Motor Simulation
```java
public class MotorPhysicsSimulation {
    public static void simulateMotorResponse(RevoluteJoint<Body> motorJoint, double commandedSpeed, double dt) {
        double currentSpeed = motorJoint.getJointSpeed();
        double speedError = commandedSpeed - currentSpeed;
        
        // Simulate motor acceleration characteristics
        double maxAcceleration = getMotorMaxAcceleration();
        double acceleration = Math.signum(speedError) * Math.min(Math.abs(speedError) / dt, maxAcceleration);
        
        // Apply torque based on motor curve
        double torque = calculateMotorTorque(commandedSpeed, currentSpeed);
        motorJoint.setMaximumMotorTorque(Math.abs(torque));
        motorJoint.setMotorSpeed(commandedSpeed);
    }
}
```

## Best Practices for FRC Simulation

1. **Realistic Physics**: Use appropriate mass, friction, and restitution values
2. **Performance**: Optimize iteration counts and timesteps for real-time simulation
3. **Stability**: Use fixed timesteps for consistent physics behavior
4. **Collision**: Configure proper collision layers to avoid unnecessary calculations
5. **Memory**: Clean up inactive physics objects regularly
6. **Integration**: Ensure physics coordinates match WPILib coordinate system

## Troubleshooting

1. **Unstable Simulation**: Reduce timestep or increase solver iterations
2. **Poor Performance**: Lower physics frequency or reduce collision complexity
3. **Unrealistic Behavior**: Check mass, friction, and constraint parameters
4. **Penetration Issues**: Adjust continuous collision detection settings
5. **Joint Problems**: Verify joint anchor points and constraint limits

## Additional Resources
- [Dyn4j Official Documentation](https://dyn4j.org/documentation/)
- [Dyn4j GitHub Repository](https://github.com/dyn4j/dyn4j)
- [Physics Simulation Best Practices](https://dyn4j.org/2010/04/rigid-body-physics-engines/)
- [2D Physics Engine Concepts](https://dyn4j.org/2010/04/about-physics-engines/)