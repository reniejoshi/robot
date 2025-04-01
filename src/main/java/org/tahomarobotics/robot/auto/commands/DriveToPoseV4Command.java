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

package org.tahomarobotics.robot.auto.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.vision.Vision;
import org.tinylog.Logger;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import static org.tahomarobotics.robot.auto.AutonomousConstants.*;

public class DriveToPoseV4Command extends Command {
    // -- Requirements --

    private final Chassis chassis = Chassis.getInstance();

    // -- Controllers --

    private final ProfiledPIDController x, y, r;

    // -- State --

    private Pose2d startPose;

    private final int isolationTarget;
    private final double blendingDistance;

    private int targetWaypoint = 0;
    private final List<Pose2d> waypoints;

    // -- Initialization --

    public DriveToPoseV4Command(int isolationTarget, double blendingDistance, Pose2d... waypoints) {
        this.isolationTarget = isolationTarget;
        this.waypoints = Arrays.stream(waypoints).filter(Objects::nonNull).toList();
        this.blendingDistance = blendingDistance;

        x = new ProfiledPIDController(
            TRANSLATION_ALIGNMENT_KP, TRANSLATION_ALIGNMENT_KI, TRANSLATION_ALIGNMENT_KD,
            TRANSLATION_ALIGNMENT_CONSTRAINTS
        );
        x.setTolerance(0);

        y = new ProfiledPIDController(
            TRANSLATION_ALIGNMENT_KP, TRANSLATION_ALIGNMENT_KI, TRANSLATION_ALIGNMENT_KD,
            TRANSLATION_ALIGNMENT_CONSTRAINTS
        );
        y.setTolerance(0);

        r = new ProfiledPIDController(
            ROTATION_ALIGNMENT_KP, ROTATION_ALIGNMENT_KI, ROTATION_ALIGNMENT_KD,
            ROTATION_ALIGNMENT_CONSTRAINTS
        );
        r.setTolerance(ROTATION_ALIGNMENT_TOLERANCE);
        r.enableContinuousInput(-Math.PI, Math.PI);

        syncGoal();
        addRequirements(chassis);
    }

    // -- Command --

    @Override
    public void initialize() {
        Pose2d currentPose = chassis.getPose();
        ChassisSpeeds currentVelocity = chassis.getFieldChassisSpeeds();

        Logger.info("Driving to {} from {} going through {}", waypoints.get(0), waypoints.subList(1, waypoints.size()), currentPose);

        x.reset(currentPose.getX(), currentVelocity.vxMetersPerSecond);
        y.reset(currentPose.getY(), currentVelocity.vyMetersPerSecond);
        r.reset(currentPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond);
        syncGoal();

        chassis.setAutoAligning(true);
        Vision.getInstance().isolate(isolationTarget);

        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/Waypoints", waypoints.toArray(Pose2d[]::new));

        startPose = currentPose;
    }

    @Override
    public void execute() {
        Pose2d currentPose = chassis.getPose();

        double distanceToGoalPose = getDistanceToWaypoint();
        if (distanceToGoalPose < blendingDistance && targetWaypoint < waypoints.size() - 1) {
            targetWaypoint++;
            syncGoal();

            Logger.info("Transition to waypoint {}", targetWaypoint);
        }

        double vx = x.calculate(currentPose.getX());
        double vy = y.calculate(currentPose.getY());
        double vr = r.calculate(currentPose.getRotation().getRadians());

        chassis.drive(new ChassisSpeeds(vx, vy, vr), true);

        // Logging
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/X Distance", getVerticalDistanceToWaypoint());
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/Y Distance", getHorizontalDistanceToWaypoint());
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/Target Waypoint", waypoints.get(targetWaypoint));

        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/vx", vx);
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/vy", vy);
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/vr", vr);
    }

    @Override
    public boolean isFinished() {
        Translation2d robotToTarget = chassis.getPose().getTranslation().minus(waypoints.get(targetWaypoint).getTranslation());
        return (Math.abs(robotToTarget.getX()) < X_TOLERANCE
                && Math.abs(robotToTarget.getY()) < Y_TOLERANCE
                && r.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
        chassis.setAutoAligning(false);

        Vision.getInstance().globalize();

        Logger.info("Finished drive to pose command" + ((interrupted) ? " because it was interrupted." : "!"));
    }

    // -- Instance Methods --

    public double getDistanceFromStart() {
        return startPose.getTranslation().getDistance(chassis.getPose().getTranslation());
    }

    public double getDistanceToWaypoint() {
        return waypoints.get(targetWaypoint).getTranslation().getDistance(chassis.getPose().getTranslation());
    }

    public double getAngleToWaypoint() {
        return waypoints.get(targetWaypoint).getRotation().getRadians() - chassis.getPose().getRotation().getRadians();
    }

    public double getHorizontalDistanceToWaypoint() {
        return getDistanceToWaypoint() * Math.cos(getAngleToWaypoint());
    }

    public double getVerticalDistanceToWaypoint() {
        return getDistanceToWaypoint() * Math.sin(getAngleToWaypoint());
    }

    public int getTargetWaypoint() {
        return targetWaypoint;
    }

    // -- Command Helpers --

    public Command runWhen(BooleanSupplier condition, Command command) {
        return Commands.waitUntil(condition).andThen(command);
    }

    // -- Helpers --

    private void syncGoal() {
        Pose2d goalPose = waypoints.get(targetWaypoint);

        x.setGoal(goalPose.getX());
        y.setGoal(goalPose.getY());
        r.setGoal(goalPose.getRotation().getRadians());
    }
}