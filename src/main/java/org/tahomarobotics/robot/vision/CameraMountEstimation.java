/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.tinylog.Logger;

import java.util.Map;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static org.tahomarobotics.robot.vision.VisionConstants.FIELD_LAYOUT;

public class CameraMountEstimation {
    public static Consumer<AprilTagCamera.EstimatedRobotPose> stream(Consumer<AprilTagCamera.EstimatedRobotPose> original) {
        // Whether to estimate the next AprilTag update from each camera.
        Map<String, Boolean> estimate =
            Stream.of(VisionConstants.ELEVATOR_SWERVE, VisionConstants.CLIMBER_SWERVE)
                  .collect(Collectors.toMap(VisionConstants.CameraConfiguration::name, $ -> false));

        // Initialize all of our inputs on SmartDashboard
        SmartDashboard.putData(
            "Estimate Camera Positions", Commands.runOnce(() -> estimate.replaceAll((n, v) -> true)).ignoringDisable(true)
        );

        SmartDashboard.putNumber("Actual Chassis Pose X (Meters)", 0);
        SmartDashboard.putNumber("Actual Chassis Pose Y (Meters)", 0);
        SmartDashboard.putNumber("Actual Chassis Pose Heading (Degrees)", 0);

        return estimatedRobotPose -> {
            if (estimate.getOrDefault(estimatedRobotPose.cameraName(), false)) {
                double x = SmartDashboard.getNumber("Actual Chassis Pose X (Meters)", 0);
                double y = SmartDashboard.getNumber("Actual Chassis Pose Y (Meters)", 0);
                double h = Units.degreesToRadians(SmartDashboard.getNumber("Actual Chassis Pose Heading (Degrees)", 0));

                Pose2d actualChassisPose = new Pose2d(x, y, new Rotation2d(h));

                for (PhotonTrackedTarget target : estimatedRobotPose.targets()) {
                    // Get the camera-to-target transform
                    Transform3d cameraToTargetTranspose = target.getBestCameraToTarget();

                    // Get the expected field-pose of the corresponding apriltag on the field
                    Pose3d targetPose = FIELD_LAYOUT.getTagPose(target.getFiducialId()).orElseThrow();

                    // Subtract the camera-to-target transform from the target field-pose to get the expected field-to-camera position
                    Pose3d cameraPose = targetPose.plus(cameraToTargetTranspose.inverse());

                    // Subtract the field-to-camera pose from the actual field-to-chassis pose to get the expected camera-to-chassis transform
                    Transform3d cameraToRobotTransform = cameraPose.minus(new Pose3d(actualChassisPose));

                    // Decompose the transformation into it's components
                    Translation3d t = cameraToRobotTransform.getTranslation();
                    Rotation3d r = cameraToRobotTransform.getRotation();

                    // Publish the results to SmartDashboard
                    SmartDashboard.putNumberArray(
                        "Camera Mount Estimation (Inches and Degrees)/" + estimatedRobotPose.cameraName(), new double[]{
                            Units.metersToInches(t.getX()),
                            Units.metersToInches(t.getY()),
                            Units.metersToInches(t.getZ()),
                            Units.radiansToDegrees(r.getX()),
                            Units.radiansToDegrees(r.getY()),
                            Units.radiansToDegrees(r.getZ())}
                    );
                }

                estimate.put(estimatedRobotPose.cameraName(), false);
            }
            original.accept(estimatedRobotPose);
        };
    }
}
