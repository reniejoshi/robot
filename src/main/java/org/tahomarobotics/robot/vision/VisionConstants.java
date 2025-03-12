package org.tahomarobotics.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.photonvision.simulation.SimCameraProperties;
import org.tahomarobotics.robot.chassis.ChassisConstants;

/** Constants for the {@link Vision} subsystem. */
public class VisionConstants {
    // AprilTag Field Layout

    // TODO: Convert this to be a mapping from field name (or some other identifier) to a layout so we can support
    //  multiple fields.
    /**
     * The AprilTag field layout for whatever field we are on. See {@link AprilTagCamera}'s JavaDoc for more information
     * on how to calculate this properly.
     */
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Cameras

    public final static CameraConfiguration ELEVATOR_SWERVE = new CameraConfiguration(
        "Elevator Swerve",
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-9.070), Units.inchesToMeters(10.573), Units.inchesToMeters(7.555)),
            new Rotation3d(Units.degreesToRadians(-2.293), Units.degreesToRadians(-7.377), Units.degreesToRadians(-163.829))
        ),
        StandardDeviationScaling.DEFAULT
    );

    public final static CameraConfiguration CLIMBER_SWERVE = new CameraConfiguration(
        "Climber Swerve",
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-9.43), Units.inchesToMeters(-12.801), Units.inchesToMeters(8.115)),
            new Rotation3d(Units.degreesToRadians(3.19), Units.degreesToRadians(-10.819), Units.degreesToRadians(162.719))
        ),
        StandardDeviationScaling.DEFAULT
    );

    public final static CameraConfiguration LIME_LIGHT = new CameraConfiguration(
        "LimeLight",
        new Transform3d(
            new Translation3d(Units.inchesToMeters(6.1875), Units.inchesToMeters(ChassisConstants.HALF_TRACK_WIDTH - 1.5), 0.81),
            new Rotation3d(0, Units.degreesToRadians(-34), 0)
        ),
        StandardDeviationScaling.DEFAULT
    );

    // Standard Deviations

    public static final Vector<N3> BASE_MULTI_TAG_STD_DEV = VecBuilder.fill(0.5, 0.5, 1);
    public static final Vector<N3> BASE_SINGLE_TAG_STD_DEV = VecBuilder.fill(1.5, 1.5, 2);
    public static final Vector<N3> BASE_ISOLATED_SINGLE_TAG_STD_DEV = VecBuilder.fill(0.125, 0.125, 10);

    public static final double AMBIGUITY_THRESHOLD = 0.03;
    public static final double MAX_CORAL_DISTANCE = 3;

    // Constraint Punishment

    public static final boolean HEADING_FREE = false;
    public static final double GYRO_ERROR_SCALING_FACTOR = 500.0;

    // Simulation

    public static final SimCameraProperties simOV9782Properties = new SimCameraProperties() {{
        setCalibration(
            1280,
            720,
            MatBuilder.fill(
                Nat.N3(), Nat.N3(),
                895.0681882368845, 0.0, 711.9376583910714,
                0.0, 896.6336103968874, 333.5574273453275,
                0.0, 0.0, 1.0
            ),
            VecBuilder.fill(
                0.011040036794979738,
                0.025690451227094003,
                0.0012670750613393597,
                -1.079822477748635E-4,
                -0.05583469833028936,
                5.147188640387755E-4,
                6.085269216455457E-4,
                0.003908226961469329
            )
        );
        setCalibError(0.35, 0.10);
        setFPS(30);
        setAvgLatencyMs(30);
        setLatencyStdDevMs(10);
    }};

    // Camera Configuration

    public record CameraConfiguration(String name, Transform3d transform, StandardDeviationScaling stdDevScaling) {}

    public interface StandardDeviationScaling {
        Vector<N3> scaleStandardDeviations(Vector<N3> stdDevs, double distance, int targetCount);

        StandardDeviationScaling DEFAULT = (stdDevs, distance, targetCount) -> stdDevs;
    }
}
