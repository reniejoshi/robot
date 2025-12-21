# PhotonVision API Documentation

## Overview
PhotonVision is a free, fast, and easy-to-use computer vision solution for FRC robots. It provides real-time target detection, pose estimation, and camera calibration capabilities designed specifically for FRC competition scenarios.

## Version Information
- **Library Version**: v2025.3.1
- **Java Group ID**: org.photonvision
- **Artifacts**: 
  - photonlib-java
  - photontargeting-java
- **Maven URL**: https://maven.photonvision.org/repository/internal

## Core Components

### PhotonCamera
Interface for communicating with PhotonVision cameras.

```java
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

// Create camera instance
PhotonCamera camera = new PhotonCamera("photonvision");

// Get latest results
var result = camera.getLatestResult();

if (result.hasTargets()) {
    PhotonTrackedTarget target = result.getBestTarget();
    
    // Get target information
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    double skew = target.getSkew();
    
    // Get pose ambiguity for AprilTags
    double poseAmbiguity = target.getPoseAmbiguity();
}
```

### PhotonPoseEstimator
Combines AprilTag detections with robot kinematics for pose estimation.

```java
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// Load AprilTag field layout
AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

// Configure pose estimator
PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
    fieldLayout,
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    camera,
    robotToCamera // Transform3d from robot center to camera
);

// Get pose estimate
var poseResult = poseEstimator.update();
if (poseResult.isPresent()) {
    EstimatedRobotPose estimatedPose = poseResult.get();
    Pose2d robotPose = estimatedPose.estimatedPose.toPose2d();
    double timestamp = estimatedPose.timestampSeconds;
}
```

## AprilTag Integration

### Field Layout Configuration
```java
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

// Use official field layout
AprilTagFieldLayout fieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

// Or load custom layout
AprilTagFieldLayout customLayout = new AprilTagFieldLayout(
    "path/to/custom/field/layout.json"
);
```

### Pose Estimation Strategies
```java
// Different strategies for pose estimation
PoseStrategy.LOWEST_AMBIGUITY          // Use tag with lowest pose ambiguity
PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT  // Use tag closest to camera height
PoseStrategy.CLOSEST_TO_REFERENCE_POSE // Use tag closest to reference pose
PoseStrategy.CLOSEST_TO_LAST_POSE     // Use tag closest to last known pose
PoseStrategy.AVERAGE_BEST_TARGETS     // Average multiple good targets
PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR // Use multi-tag PnP (recommended)
```

## Vision Processing Pipeline

### Target Detection
```java
// Configure pipeline for different game elements
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera aprilTagCamera;
    private final PhotonCamera gameElementCamera;
    
    public VisionSubsystem() {
        aprilTagCamera = new PhotonCamera("apriltag-camera");
        gameElementCamera = new PhotonCamera("game-element-camera");
    }
    
    @Override
    public void periodic() {
        // Process AprilTag results
        processAprilTagResults();
        
        // Process game element results
        processGameElementResults();
    }
    
    private void processAprilTagResults() {
        var result = aprilTagCamera.getLatestResult();
        
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                int fiducialId = target.getFiducialId();
                Transform3d robotToTarget = target.getBestCameraToTarget();
                
                // Process specific AprilTag
                processAprilTag(fiducialId, robotToTarget);
            }
        }
    }
}
```

### Camera Calibration Integration
```java
// Camera intrinsics are automatically loaded from PhotonVision
// Calibration data is stored on the coprocessor and used for:
// - Pose estimation accuracy
// - Distance calculations
// - 3D position reconstruction
```

## Odometry Fusion

### Vision-Assisted Odometry
```java
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class DriveSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final PhotonPoseEstimator photonPoseEstimator;
    
    @Override
    public void periodic() {
        // Update odometry with encoder data
        poseEstimator.update(
            getRotation(),
            getModulePositions()
        );
        
        // Add vision measurements
        var visionResult = photonPoseEstimator.update();
        if (visionResult.isPresent()) {
            EstimatedRobotPose visionPose = visionResult.get();
            
            // Add vision measurement with appropriate standard deviations
            poseEstimator.addVisionMeasurement(
                visionPose.estimatedPose.toPose2d(),
                visionResult.timestampSeconds,
                VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5)) // std devs
            );
        }
    }
}
```

## Simulation Support

### Simulated Vision
```java
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSimulation {
    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;
    
    public VisionSimulation() {
        // Create vision system simulation
        visionSim = new VisionSystemSim("main");
        
        // Add field AprilTags
        visionSim.addAprilTags(fieldLayout);
        
        // Configure camera properties
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(1280, 720, Rotation2d.fromDegrees(70));
        cameraProperties.setCalibError(0.25, 0.08);
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);
        
        // Create camera simulation
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);
    }
    
    public void updateSimulation(Pose2d robotPose) {
        visionSim.update(robotPose);
    }
}
```

## Networking and Communication

### NetworkTables Integration
PhotonVision automatically publishes data to NetworkTables:

```java
// Data is published to /photonvision/{camera-name}/
// - hasTarget: boolean
// - targetYaw: double
// - targetPitch: double  
// - targetArea: double
// - targetPose: double array (if AprilTag)
// - pipelineIndex: int
```

### Camera Configuration
```java
// Set pipeline from robot code
camera.setPipelineIndex(0); // AprilTag pipeline
camera.setPipelineIndex(1); // Game element pipeline

// Get current pipeline
int currentPipeline = camera.getPipelineIndex();
```

## Coordinate Systems

### Transform3d Usage
```java
// Define camera position relative to robot center
Transform3d robotToCamera = new Transform3d(
    new Translation3d(0.3, 0.0, 0.2), // 30cm forward, 20cm up
    new Rotation3d(0, Units.degreesToRadians(-15), 0) // 15° down
);

// Camera to target transform (from PhotonVision)
Transform3d cameraToTarget = target.getBestCameraToTarget();

// Calculate robot to target
Transform3d robotToTarget = robotToCamera.plus(cameraToTarget);
```

## Performance Optimization

### Pipeline Management
```java
public class VisionManager {
    private final PhotonCamera camera;
    private final Timer pipelineTimer = new Timer();
    
    public void managePipelines() {
        // Switch pipelines based on game state
        if (DriverStation.isAutonomous()) {
            camera.setPipelineIndex(APRILTAG_PIPELINE);
        } else if (needGameElement()) {
            camera.setPipelineIndex(GAME_ELEMENT_PIPELINE);
        }
    }
    
    // Reduce processing frequency when not needed
    public void setProcessingEnabled(boolean enabled) {
        if (enabled) {
            camera.setDriverMode(false);
        } else {
            camera.setDriverMode(true); // Reduces CPU usage
        }
    }
}
```

## Integration Patterns Used in This Codebase

Based on the import analysis, this robot uses PhotonVision for:

- **Vision Subsystem**: Primary camera interface and target detection
- **Vision Simulation**: Physics-based camera simulation for testing
- **Pose Estimation**: Integration with odometry for accurate localization

## Common Use Cases

### Auto-Alignment
```java
public Command alignToTarget() {
    return run(() -> {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            double rotationSpeed = -alignPID.calculate(target.getYaw(), 0);
            drive(0, 0, rotationSpeed);
        }
    }).until(() -> Math.abs(getTargetYaw()) < 2.0);
}
```

### Distance Calculation
```java
public double getDistanceToTarget() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        
        return PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(target.getPitch())
        );
    }
    return 0.0;
}
```

## Best Practices

1. **Camera Placement**: Mount cameras with clear view of targets and minimal vibration
2. **Lighting Conditions**: Test vision system under various lighting conditions
3. **Pipeline Tuning**: Tune vision pipelines for specific field conditions
4. **Latency Compensation**: Account for camera and network latency in controls
5. **Redundancy**: Use multiple cameras for critical vision tasks
6. **Simulation**: Test vision code extensively in simulation
7. **Pose Validation**: Validate vision poses against odometry estimates

## Troubleshooting

1. **No Targets Detected**: Check lighting, pipeline settings, and camera position
2. **Inaccurate Poses**: Verify camera calibration and robot-to-camera transform
3. **High Latency**: Reduce resolution, FPS, or processing complexity
4. **NetworkTables Issues**: Check camera name and network configuration
5. **Simulation Problems**: Verify field layout and camera properties

## Additional Resources
- [PhotonVision Documentation](https://docs.photonvision.org/)
- [AprilTag Field Layouts](https://github.com/wpilibsuite/allwpilib/tree/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag)
- [Camera Calibration Guide](https://docs.photonvision.org/en/latest/docs/calibration/calibration.html)
- [PhotonVision GitHub](https://github.com/PhotonVision/photonvision)