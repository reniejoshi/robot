# PathPlannerLib API Documentation

## Overview
PathPlannerLib is a motion planning library designed specifically for FRC robots. It provides path planning, trajectory generation, and path following capabilities with a focus on ease of use and integration with WPILib.

## Version Information
- **Library Version**: 2025.2.7
- **Java Group ID**: com.pathplanner.lib
- **Artifact**: PathplannerLib-java
- **Maven URL**: https://3015rangerrobotics.github.io/pathplannerlib/repo

## Core Components

### PathPlannerPath
Represents a path created in the PathPlanner GUI application.

```java
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;

// Load a path from the deploy directory
PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

// Create constraints for path following
PathConstraints constraints = new PathConstraints(
    3.0, // Max velocity (m/s)
    3.0, // Max acceleration (m/s²)
    Units.degreesToRadians(540), // Max angular velocity (rad/s)
    Units.degreesToRadians(720)  // Max angular acceleration (rad/s²)
);
```

### PathPlannerTrajectory
Generated trajectory from a PathPlannerPath with specific constraints and robot configuration.

```java
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

// Generate trajectory from path
PathPlannerTrajectory trajectory = path.getTrajectory(
    new ChassisSpeeds(), // Starting robot-relative speeds
    robotPose.getRotation() // Starting robot rotation
);
```

### AutoBuilder
Simplifies autonomous routine creation and command generation.

```java
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given robot relative ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters
            new ReplanningConfig() // Default path replanning config
        ),
        () -> {
            // Boolean supplier that controls when the path will be mirrored
            // This will flip the path being followed to the red side of the field
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}
```

### PathFollowing Commands
Generate commands to follow paths and trajectories.

```java
// Follow a single path
Command followPathCommand = AutoBuilder.followPath(path);

// Follow a path with event markers
Command pathWithEvents = AutoBuilder.followPathWithEvents(path);

// Create a full autonomous routine
Command autoCommand = AutoBuilder.buildAuto("My Auto");
```

## Path Creation and Management

### PathPlanner GUI Application
- **Installation**: Download from GitHub releases
- **File Format**: JSON files stored in `src/main/deploy/pathplanner/paths/`
- **Features**:
  - Visual path editing
  - Waypoint management
  - Velocity constraints
  - Event markers
  - Path previsualization

### Programmatic Path Creation
```java
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

// Create path from waypoints
List<PathPoint> waypoints = List.of(
    new PathPoint(new Translation2d(0, 0), null, new Rotation2d()),
    new PathPoint(new Translation2d(3, 0), null, new Rotation2d()),
    new PathPoint(new Translation2d(6, 3), null, new Rotation2d(Math.PI))
);

PathPlannerPath path = new PathPlannerPath(
    waypoints,
    constraints,
    new GoalEndState(0.0, Rotation2d.fromDegrees(-90))
);
```

## Swerve Drive Integration

### SwerveAutoBuilder (Legacy)
```java
import com.pathplanner.lib.auto.SwerveAutoBuilder;

// Configure auto builder for swerve drive
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    swerve::getPose, // Pose2d supplier
    swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    swerve.kinematics, // SwerveDriveKinematics
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error
    swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap, // Event map
    true, // Should the path be automatically mirrored depending on alliance color
    swerve // The drive subsystem
);
```

### Modern Swerve Integration
```java
// In your swerve drive subsystem
public void configurePathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::drive,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            MAX_SPEED_METERS_PER_SECOND,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig()
        ),
        this::shouldFlipPath,
        this
    );
}
```

## Event Markers and Named Commands

### Named Commands
```java
import com.pathplanner.lib.auto.NamedCommands;

// Register named commands for use in paths
NamedCommands.registerCommand("intake", intakeCommand);
NamedCommands.registerCommand("shoot", shootCommand);
NamedCommands.registerCommand("deploy", deployCommand);
```

### Event Markers in Paths
Event markers can be placed in the PathPlanner GUI and will automatically execute the corresponding named commands during path following.

## Coordinate System and Field Layout

### Field Coordinate System
PathPlannerLib uses the WPILib coordinate system:
- **Origin**: Blue alliance driver station wall
- **X-axis**: Points toward red alliance
- **Y-axis**: Points toward center of field (left when facing red alliance)
- **Rotation**: Counter-clockwise positive, 0° is toward red alliance

### Alliance Color Handling
```java
// Automatically mirror paths for red alliance
public boolean shouldFlipPath() {
    return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red)
        .orElse(false);
}
```

## Path Constraints and Optimization

### Velocity Constraints
```java
// Define constraints for different path segments
PathConstraints fastConstraints = new PathConstraints(4.0, 3.0, 
    Units.degreesToRadians(540), Units.degreesToRadians(720));
    
PathConstraints slowConstraints = new PathConstraints(2.0, 2.0,
    Units.degreesToRadians(360), Units.degreesToRadians(540));
```

### Dynamic Constraints
```java
// Apply different constraints based on robot state
PathConstraints getConstraints() {
    if (hasGamePiece()) {
        return slowConstraints; // Slower when carrying game piece
    }
    return fastConstraints;
}
```

## Telemetry and Debugging

### Path Visualization
```java
// Log current path for visualization
field.getObject("trajectory").setTrajectory(currentTrajectory);

// Log target pose
field.getObject("target").setPose(targetPose);
```

### Performance Monitoring
```java
// Monitor path following errors
SmartDashboard.putNumber("Path/Translation Error", translationError);
SmartDashboard.putNumber("Path/Rotation Error", rotationError);
SmartDashboard.putNumber("Path/Velocity Error", velocityError);
```

## Integration Patterns Used in This Codebase

Based on the import analysis, this robot likely uses PathPlannerLib for:

- **Autonomous Navigation**: Pre-planned paths for scoring and game piece collection
- **Chassis Control**: Integration with swerve drive system
- **Test Routines**: Automated testing of drivetrain and subsystems

## Common File Structure
```
src/main/deploy/pathplanner/
├── autos/
│   ├── Example Auto.auto
│   └── Score and Leave.auto
├── navgrid.json
└── paths/
    ├── Example Path.path
    ├── Score High.path
    └── Leave Community.path
```

## Best Practices

1. **Path Design**: Create smooth, efficient paths with appropriate constraints
2. **Event Timing**: Use event markers for precise command execution during path following
3. **Testing**: Test paths in simulation before deploying to robot
4. **Alliance Handling**: Always account for alliance color mirroring
5. **Constraints**: Tune constraints based on robot capabilities and field conditions
6. **Visualization**: Use field2d widgets to visualize paths and robot position

## Troubleshooting

1. **Path Not Found**: Ensure path files are in correct deploy directory
2. **Mirroring Issues**: Check alliance color detection logic
3. **Following Errors**: Tune PID constants for translation and rotation
4. **Performance**: Monitor CPU usage and reduce path complexity if needed
5. **Coordinate System**: Verify field coordinate system matches WPILib conventions

## Additional Resources
- [PathPlannerLib Documentation](https://pathplanner.dev/)
- [PathPlanner GUI Releases](https://github.com/mjansen4857/pathplanner/releases)
- [WPILib Coordinate System](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html)