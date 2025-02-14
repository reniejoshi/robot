package org.tahomarobotics.robot.windmill;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.IntStream;

public class WindmillTrajectory {
    private final Trajectory trajectory;
    private final List<Direction> directions;

    private WindmillState previousState;

    public WindmillTrajectory(
        TrajectoryConfig config, List<Direction> directions, List<Translation2d> points, List<Translation2d> tangents
    ) {
        assert !points.isEmpty();
        assert points.size() == tangents.size(); // TODO: Throw checked exception instead

        int last = points.size() - 1;
        var start = new Spline.ControlVector(
            new double[]{points.get(0).getX(), tangents.get(0).getX()},
            new double[]{points.get(0).getY(), tangents.get(0).getY()}
        );
        var end = new Spline.ControlVector(
            new double[]{points.get(last).getX(), tangents.get(last).getX()},
            new double[]{points.get(last).getY(), tangents.get(last).getY()}
        );
        var midpoints = new ArrayList<Translation2d>();

        for (int i = 1; i < last; i++) {
            midpoints.add(new Translation2d(points.get(i).getX(), points.get(i).getY()));
        }

        this.directions = directions;
        trajectory = TrajectoryGenerator.generateTrajectory(
            start, midpoints, end, config
        );
    }

    public WindmillState sample(double t) throws WindmillKinematics.KinematicsException {
        Trajectory.State state = trajectory.sample(t);
        boolean isUp = false; // The default value will never be used.
        for (int i = 1; i <= this.directions.size(); i++) {
            if (i == directions.size() || directions.get(i).time > t) {
                isUp = directions.get(i - 1).isUp;
                break;
            }
        }
        previousState = WindmillKinematics.inverseKinematics(t, state.poseMeters.getTranslation(), previousState, isUp);
        return previousState;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public double getTotalTimeSeconds() {
        return trajectory.getTotalTimeSeconds();
    }

    public Pose2d samplePose2d(double t) {
        return trajectory.sample(t).poseMeters;
    }

    public static WindmillTrajectory generateFromNetworkTables() {
        TrajectoryConfig config = new TrajectoryConfig(
            SmartDashboard.getNumber("BEEF/Config/Max Velocity", 0),
            SmartDashboard.getNumber("BEEF/Config/Max Acceleration", 0)
        );
        var directions = (IntStream.range(0, (int) SmartDashboard.getNumber("BEEF/Directions/Count", 0)))
            .mapToObj(i -> {
                String prefix = "BEEF/Directions/" + i + "/";
                return new Direction(
                    SmartDashboard.getNumber(prefix + "Time", 0),
                    Objects.equals(SmartDashboard.getString(prefix + "Direction", ""), "up")
                );
            }).toList();
        var points = (IntStream.range(0, (int) SmartDashboard.getNumber("BEEF/Points/Count", 0)))
            .mapToObj(i -> {
                String prefix = "BEEF/Points/" + i + "/Position/";
                return new Translation2d(
                    SmartDashboard.getNumber(prefix + "x", 0),
                    SmartDashboard.getNumber(prefix + "y", 0)
                );
            }).toList();
        var tangents = (IntStream.range(0, (int) SmartDashboard.getNumber("BEEF/Points/Count", 0)))
            .mapToObj(i -> {
                String prefix = "BEEF/Points/" + i + "/Tangent/";
                return new Translation2d(
                    SmartDashboard.getNumber(prefix + "x", 0),
                    SmartDashboard.getNumber(prefix + "y", 0)
                );
            }).toList();

        return new WindmillTrajectory(config, directions, points, tangents);
    }

    public static class Direction {
        public double time;
        public Boolean isUp;

        public Direction(double time, Boolean isUp) {
            this.time = time;
            this.isUp = isUp;
        }
    }
}
