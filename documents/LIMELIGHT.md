# Limelight Vision System Documentation

## Overview
Limelight is a plug-and-play vision system designed specifically for FRC robotics. It provides real-time target detection, tracking, and pose estimation with minimal setup required. Limelight cameras come pre-configured with powerful processing capabilities and easy NetworkTables integration.

## Version Information
- **Hardware**: Limelight 3/3G (latest generation)
- **Software**: Integrated web interface and NetworkTables API
- **Processing**: Onboard computer vision processing
- **Network**: Ethernet connection to robot radio/switch

## Hardware Overview

### Limelight 3/3G Features
- **Dual Cameras**: Color camera + global shutter monochrome
- **Processing**: Dedicated vision coprocessor
- **LEDs**: Integrated controllable LED array
- **Connectivity**: Ethernet (Power over Ethernet capable)
- **Mounting**: Standard FRC mounting patterns

### Network Configuration
```bash
# Default Limelight IP addresses
# Limelight 3: 10.TE.AM.11 (where TEAM is your team number)
# Example for team 2046: 10.20.46.11
```

## NetworkTables Integration

### Basic NetworkTables API
```java
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightHelper {
    private static final NetworkTable limelight = 
        NetworkTableInstance.getDefault().getTable("limelight");
    
    // Target detection
    public static boolean hasValidTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }
    
    // Target position (degrees)
    public static double getTargetX() {
        return limelight.getEntry("tx").getDouble(0.0); // -27 to +27 degrees
    }
    
    public static double getTargetY() {
        return limelight.getEntry("ty").getDouble(0.0); // -20.5 to +20.5 degrees
    }
    
    // Target area (0-100% of image)
    public static double getTargetArea() {
        return limelight.getEntry("ta").getDouble(0.0);
    }
    
    // Target skew/rotation (-90 to 0 degrees)
    public static double getTargetSkew() {
        return limelight.getEntry("ts").getDouble(0.0);
    }
    
    // Latency information (milliseconds)
    public static double getLatency() {
        return limelight.getEntry("tl").getDouble(0.0);
    }
}
```

### Advanced NetworkTables Entries
```java
public class AdvancedLimelightHelper {
    // AprilTag specific data (Limelight 3)
    public static int getAprilTagID() {
        return (int) limelight.getEntry("tid").getInteger(-1);
    }
    
    // 3D pose information (if available)
    public static double[] getBotpose() {
        return limelight.getEntry("botpose").getDoubleArray(new double[6]);
    }
    
    public static double[] getCameraPose() {
        return limelight.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    }
    
    // Raw corners of detected target
    public static double[] getTargetCorners() {
        return limelight.getEntry("tcornxy").getDoubleArray(new double[0]);
    }
    
    // Camera controls
    public static void setLEDMode(int mode) {
        // 0: pipeline default, 1: off, 2: blink, 3: on
        limelight.getEntry("ledMode").setNumber(mode);
    }
    
    public static void setCamMode(int mode) {
        // 0: vision processing, 1: driver camera
        limelight.getEntry("camMode").setNumber(mode);
    }
    
    public static void setPipeline(int pipeline) {
        // 0-9: pipeline number
        limelight.getEntry("pipeline").setNumber(pipeline);
    }
    
    public static void setStream(int mode) {
        // 0: standard, 1: PiP main, 2: PiP secondary
        limelight.getEntry("stream").setNumber(mode);
    }
}
```

## Vision Pipeline Configuration

### Web Interface Setup
1. **Connect**: Navigate to http://10.TE.AM.11:5801 in browser
2. **Calibration**: Perform camera calibration for accurate measurements
3. **Pipeline Setup**: Configure up to 10 different vision pipelines
4. **Tuning**: Adjust HSV thresholds, area filters, and contour processing

### Pipeline Types
- **Retroreflective Targets**: High-reflectivity tape detection
- **AprilTags**: Fiducial marker detection and pose estimation
- **Color Detection**: HSV-based color target tracking
- **Neural Networks**: AI-based object detection (Limelight 3)

### Pipeline Configuration Example
```java
public class VisionPipelines {
    public static final int APRILTAG_PIPELINE = 0;
    public static final int RETROREFLECTIVE_PIPELINE = 1;
    public static final int GAME_PIECE_PIPELINE = 2;
    
    public static void switchToAprilTagMode() {
        LimelightHelper.setPipeline(APRILTAG_PIPELINE);
        LimelightHelper.setLEDMode(0); // Use pipeline default
    }
    
    public static void switchToGamePieceMode() {
        LimelightHelper.setPipeline(GAME_PIECE_PIPELINE);
        LimelightHelper.setLEDMode(3); // LEDs on for color detection
    }
    
    public static void switchToDriverMode() {
        LimelightHelper.setCamMode(1); // Driver camera mode
        LimelightHelper.setLEDMode(1); // LEDs off
    }
}
```

## Distance and Pose Calculation

### Distance Calculation (Retroreflective)
```java
public class LimelightMath {
    // Camera mounting parameters
    private static final double CAMERA_HEIGHT_METERS = 0.5; // 50cm high
    private static final double TARGET_HEIGHT_METERS = 2.64; // Upper hub height
    private static final double CAMERA_ANGLE_RADIANS = Math.toRadians(30); // 30° up
    
    public static double getDistanceToTarget() {
        if (!LimelightHelper.hasValidTarget()) {
            return 0.0;
        }
        
        double targetOffsetAngle_Vertical = LimelightHelper.getTargetY();
        double angleToGoalRadians = CAMERA_ANGLE_RADIANS + Math.toRadians(targetOffsetAngle_Vertical);
        
        return (TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS) / Math.tan(angleToGoalRadians);
    }
    
    public static double getTargetDirection() {
        return Math.toRadians(LimelightHelper.getTargetX());
    }
}
```

### AprilTag Pose Estimation
```java
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTagPoseEstimation {
    public static Pose2d getRobotPoseFromAprilTag() {
        if (!LimelightHelper.hasValidTarget()) {
            return null;
        }
        
        // Get botpose from Limelight (field coordinates)
        double[] botpose = LimelightHelper.getBotpose();
        
        if (botpose.length >= 6) {
            return new Pose2d(
                botpose[0], // X position
                botpose[1], // Y position  
                Rotation2d.fromDegrees(botpose[5]) // Rotation (yaw)
            );
        }
        
        return null;
    }
    
    public static Transform2d getCameraToTarget() {
        double[] camerapose = LimelightHelper.getCameraPose();
        
        if (camerapose.length >= 6) {
            return new Transform2d(
                new Translation2d(camerapose[0], camerapose[1]),
                Rotation2d.fromDegrees(camerapose[5])
            );
        }
        
        return null;
    }
}
```

## Subsystem Integration

### Vision Subsystem
```java
public class VisionSubsystem extends SubsystemBase {
    private boolean hasTarget = false;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetArea = 0.0;
    private Pose2d visionPose = null;
    
    @Override
    public void periodic() {
        // Update target information
        hasTarget = LimelightHelper.hasValidTarget();
        
        if (hasTarget) {
            targetX = LimelightHelper.getTargetX();
            targetY = LimelightHelper.getTargetY();
            targetArea = LimelightHelper.getTargetArea();
            
            // Update pose estimate if using AprilTags
            visionPose = AprilTagPoseEstimation.getRobotPoseFromAprilTag();
        }
        
        // Log data for telemetry
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget);
        SmartDashboard.putNumber("Vision/Target X", targetX);
        SmartDashboard.putNumber("Vision/Target Y", targetY);
        SmartDashboard.putNumber("Vision/Distance", LimelightMath.getDistanceToTarget());
        
        if (visionPose != null) {
            SmartDashboard.putNumber("Vision/Pose X", visionPose.getX());
            SmartDashboard.putNumber("Vision/Pose Y", visionPose.getY());
            SmartDashboard.putNumber("Vision/Pose Rotation", visionPose.getRotation().getDegrees());
        }
    }
    
    // Getters for other subsystems
    public boolean hasValidTarget() { return hasTarget; }
    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getDistanceToTarget() { return LimelightMath.getDistanceToTarget(); }
    public Pose2d getVisionPose() { return visionPose; }
}
```

## Auto-Alignment Commands

### Turn to Target Command
```java
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnToTargetCommand extends Command {
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final PIDController turnController;
    
    public TurnToTargetCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        this.turnController = new PIDController(0.02, 0.0, 0.0);
        
        turnController.setTolerance(2.0); // 2 degree tolerance
        turnController.enableContinuousInput(-180, 180);
        
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        if (vision.hasValidTarget()) {
            double targetX = vision.getTargetX();
            double rotationSpeed = turnController.calculate(targetX, 0.0);
            
            // Apply rotation with speed limiting
            rotationSpeed = Math.max(-0.3, Math.min(0.3, rotationSpeed));
            drive.arcadeDrive(0.0, rotationSpeed);
        } else {
            drive.stop();
        }
    }
    
    @Override
    public boolean isFinished() {
        return vision.hasValidTarget() && turnController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
```

### Drive to Target Command
```java
public class DriveToTargetCommand extends Command {
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final PIDController driveController;
    private final PIDController turnController;
    private final double targetDistance;
    
    public DriveToTargetCommand(DriveSubsystem drive, VisionSubsystem vision, double targetDistance) {
        this.drive = drive;
        this.vision = vision;
        this.targetDistance = targetDistance;
        
        this.driveController = new PIDController(1.0, 0.0, 0.0);
        this.turnController = new PIDController(0.02, 0.0, 0.0);
        
        driveController.setTolerance(0.1); // 10cm tolerance
        turnController.setTolerance(2.0);  // 2 degree tolerance
        
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        if (vision.hasValidTarget()) {
            double currentDistance = vision.getDistanceToTarget();
            double targetX = vision.getTargetX();
            
            // Calculate drive and turn speeds
            double driveSpeed = driveController.calculate(currentDistance, targetDistance);
            double turnSpeed = turnController.calculate(targetX, 0.0);
            
            // Apply speed limits
            driveSpeed = Math.max(-0.5, Math.min(0.5, driveSpeed));
            turnSpeed = Math.max(-0.3, Math.min(0.3, turnSpeed));
            
            drive.arcadeDrive(driveSpeed, turnSpeed);
        } else {
            drive.stop();
        }
    }
    
    @Override
    public boolean isFinished() {
        return vision.hasValidTarget() && 
               driveController.atSetpoint() && 
               turnController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
```

## Odometry Integration

### Vision-Enhanced Odometry
```java
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.VecBuilder;

public class PoseEstimationSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final VisionSubsystem vision;
    
    public PoseEstimationSubsystem(DriveSubsystem drive, VisionSubsystem vision) {
        this.vision = vision;
        
        this.poseEstimator = new SwerveDrivePoseEstimator(
            drive.getKinematics(),
            drive.getRotation2d(),
            drive.getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Math.toRadians(5)), // State std devs
            VecBuilder.fill(0.5, 0.5, Math.toRadians(30))   // Vision std devs
        );
    }
    
    @Override
    public void periodic() {
        // Update with encoder/gyro data
        poseEstimator.update(
            drive.getRotation2d(),
            drive.getModulePositions()
        );
        
        // Add vision measurements when available
        Pose2d visionPose = vision.getVisionPose();
        if (visionPose != null) {
            double timestamp = Timer.getFPGATimestamp() - 
                (LimelightHelper.getLatency() / 1000.0);
            
            poseEstimator.addVisionMeasurement(visionPose, timestamp);
        }
        
        // Log combined pose estimate
        Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
        field.setRobotPose(estimatedPose);
    }
}
```

## Game-Specific Applications

### Game Piece Tracking
```java
public class GamePieceTracker {
    public static final int GAME_PIECE_PIPELINE = 2;
    
    public static void enableGamePieceTracking() {
        LimelightHelper.setPipeline(GAME_PIECE_PIPELINE);
        LimelightHelper.setLEDMode(3); // LEDs on for color detection
    }
    
    public static boolean hasGamePiece() {
        return LimelightHelper.hasValidTarget() && 
               LimelightHelper.getTargetArea() > 0.5; // Minimum area threshold
    }
    
    public static double getGamePieceAngle() {
        return LimelightHelper.getTargetX();
    }
    
    public static Command createPickupCommand(DriveSubsystem drive) {
        return Commands.run(() -> {
            if (hasGamePiece()) {
                double angle = getGamePieceAngle();
                double turnSpeed = Math.max(-0.2, Math.min(0.2, angle * 0.01));
                drive.arcadeDrive(0.3, turnSpeed); // Drive forward while aligning
            } else {
                drive.stop();
            }
        }).until(() -> !hasGamePiece()); // Stop when game piece is picked up
    }
}
```

## Best Practices

### Performance Optimization
1. **Pipeline Management**: Switch pipelines based on game state
2. **LED Control**: Turn off LEDs when not needed to save power
3. **Network Traffic**: Limit unnecessary NetworkTables updates
4. **Latency Compensation**: Account for processing and network delays

### Reliability
1. **Target Validation**: Verify target characteristics before acting
2. **Fallback Modes**: Provide manual override capabilities  
3. **Connection Monitoring**: Check for Limelight connectivity
4. **Error Handling**: Gracefully handle missing or invalid data

### Competition Preparation
1. **Calibration**: Re-calibrate cameras at competition venue
2. **Lighting**: Test under various lighting conditions
3. **Field Testing**: Validate on official field elements
4. **Backup Plans**: Prepare for vision system failures

## Usage Patterns in This Codebase

Based on the search results, this robot uses Limelight for:
- **Vision Subsystem**: Target detection and pose estimation
- **Vision Simulation**: Simulated Limelight behavior for testing
- **Auto-Alignment**: Precise positioning for game piece manipulation
- **Pose Estimation**: Enhanced odometry accuracy with AprilTags

## Troubleshooting

1. **No Target Detection**: Check pipeline settings, lighting, and target visibility
2. **Inaccurate Measurements**: Verify camera calibration and mounting parameters
3. **Network Issues**: Check IP configuration and NetworkTables connection
4. **Poor Performance**: Optimize pipeline settings and reduce processing load
5. **Lighting Problems**: Adjust LED brightness and camera exposure settings

## Additional Resources
- [Limelight Documentation](https://docs.limelightvision.io/)
- [NetworkTables API Reference](https://docs.wpilib.org/en/stable/docs/software/networktables/)
- [Vision Processing Best Practices](https://docs.wpilib.org/en/stable/docs/software/vision-processing/)
- [FRC Field Coordinate System](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html)