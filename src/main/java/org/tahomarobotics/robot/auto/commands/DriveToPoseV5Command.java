package org.tahomarobotics.robot.auto.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.vision.Vision;
import org.tinylog.Logger;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static org.tahomarobotics.robot.auto.AutonomousConstants.*;

public class DriveToPoseV5Command extends Command {
    // -- Requirements --

    private final Chassis chassis = Chassis.getInstance();

    // -- Controllers --

    private final ProfiledPIDController x, y, r;

    // -- State --

    private final int isolationTarget;

    private final Supplier<Optional<Pose2d>> targetPositionSupplier;
    private Pose2d targetPose;

    // -- Initialization --

    public DriveToPoseV5Command(int isolationTarget, Supplier<Optional<Pose2d>> targetPositionSupplier, Pose2d defaultPose) {
        this.isolationTarget = isolationTarget;
        this.targetPose = targetPositionSupplier.get().orElse(defaultPose);
        this.targetPositionSupplier = targetPositionSupplier;

        x = new ProfiledPIDController(
            TRANSLATION_ALIGNMENT_KP, TRANSLATION_ALIGNMENT_KI, TRANSLATION_ALIGNMENT_KD,
            TRANSLATION_ALIGNMENT_CONSTRAINTS
        );
        x.setTolerance(TRANSLATION_ALIGNMENT_TOLERANCE);

        y = new ProfiledPIDController(
            TRANSLATION_ALIGNMENT_KP, TRANSLATION_ALIGNMENT_KI, TRANSLATION_ALIGNMENT_KD,
            TRANSLATION_ALIGNMENT_CONSTRAINTS
        );
        y.setTolerance(TRANSLATION_ALIGNMENT_TOLERANCE);

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

        Logger.info("Driving to {} from {}", targetPositionSupplier.get(), currentPose);

        x.reset(currentPose.getX(), currentVelocity.vxMetersPerSecond);
        y.reset(currentPose.getY(), currentVelocity.vyMetersPerSecond);
        r.reset(currentPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond);

        chassis.setAutoAligning(true);
        Vision.getInstance().isolate(isolationTarget);
    }

    @Override
    public void execute() {
        Pose2d currentPose = chassis.getPose();
        targetPose = targetPositionSupplier.get().orElse(targetPose);

        syncGoal();

        double distanceToGoalPose = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        double speedReduction = MathUtil.clamp(distanceToGoalPose, 0.75, 1.0);

        double vx = x.calculate(currentPose.getX()) * speedReduction;
        double vy = y.calculate(currentPose.getY()) * speedReduction;
        double vr = r.calculate(currentPose.getRotation().getRadians());

        chassis.drive(new ChassisSpeeds(vx, vy, vr), true);
    }

    @Override
    public boolean isFinished() {
        return x.atGoal() && y.atGoal() && r.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
        chassis.setAutoAligning(false);

        Vision.getInstance().globalize();
    }

    // -- Instance Methods --

    public double getDistanceToWaypoint() {
        return targetPose.getTranslation().getDistance(chassis.getPose().getTranslation());
    }

    public double getAngleToWaypoint() {
        return targetPose.getRotation().getRadians() - chassis.getPose().getRotation().getRadians();
    }

    public double getRobotHorizontalDistanceToWaypoint() {
        return getDistanceToWaypoint() * Math.cos(getAngleToWaypoint());
    }

    public double getRobotVerticalDistanceToWaypoint() {
        return getDistanceToWaypoint() * Math.sin(getAngleToWaypoint());
    }

    // -- Command Helpers --

    public Command runWhen(BooleanSupplier condition, Command command) {
        return Commands.waitUntil(condition).andThen(command);
    }

    // -- Helpers --

    private void syncGoal() {
        x.setGoal(targetPose.getX());
        y.setGoal(targetPose.getY());
        r.setGoal(targetPose.getRotation().getRadians());
    }
}