package org.tahomarobotics.robot.auto.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
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

    private final int isolationTarget;
    private final double blendingDistance;

    private double distanceToGoalPose = Double.POSITIVE_INFINITY;

    private int targetWaypoint = 0;
    private final List<Pose2d> waypoints;

    // -- Timer --
    private final Timer timer = new Timer();
    private double endTime = Double.POSITIVE_INFINITY;

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
        ChassisSpeeds currentVelocity = chassis.getChassisSpeeds();

        Logger.info("Driving to {} from {} going through {}", waypoints.get(0), waypoints.subList(1, waypoints.size()), currentPose);

        x.reset(currentPose.getX(), currentVelocity.vxMetersPerSecond);
        y.reset(currentPose.getY(), currentVelocity.vyMetersPerSecond);
        r.reset(currentPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond);

        chassis.setAutoAligning(true);
        Vision.getInstance().isolate(isolationTarget);

        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/Waypoints", waypoints.toArray(Pose2d[]::new));
    }

    @Override
    public void execute() {
        Pose2d currentPose = chassis.getPose();

        distanceToGoalPose = currentPose.getTranslation().getDistance(waypoints.get(targetWaypoint).getTranslation());
        if (distanceToGoalPose < blendingDistance && targetWaypoint < waypoints.size() - 1) {
            targetWaypoint++;
            syncGoal();

            Logger.info("Transition to waypoint {}", targetWaypoint);
        }

        double speedReduction = targetWaypoint == waypoints.size() - 1 ? MathUtil.clamp(distanceToGoalPose, 0.75, 1.0) : 1;

        double vx = x.calculate(currentPose.getX()) * speedReduction;
        double vy = y.calculate(currentPose.getY()) * speedReduction;
        double vr = r.calculate(currentPose.getRotation().getRadians());

        chassis.drive(new ChassisSpeeds(vx, vy, vr), true);

        // Logging
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/X Distance", getVerticalDistanceToWaypoint());
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/Y Distance", getHorizontalDistanceToWaypoint());
        org.littletonrobotics.junction.Logger.recordOutput("Autonomous/Drive To Pose/Target Waypoint", waypoints.get(targetWaypoint));
    }

    @Override
    public boolean isFinished() {
        Translation2d robotToTarget = chassis.getPose().getTranslation().minus(waypoints.get(targetWaypoint).getTranslation());
        return (Math.abs(robotToTarget.getX()) < X_TOLERANCE
                && Math.abs(robotToTarget.getY()) < Y_TOLERANCE
                && r.atGoal())
               || timer.hasElapsed(endTime);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
        chassis.setAutoAligning(false);

        Vision.getInstance().globalize();

        Logger.info("Finished drive to pose command!");
    }

    // -- Instance Methods --

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