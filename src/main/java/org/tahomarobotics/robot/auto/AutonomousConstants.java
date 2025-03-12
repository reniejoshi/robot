package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.tahomarobotics.robot.auto.commands.DriveToPoseV4Command;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tahomarobotics.robot.vision.VisionConstants;
import org.tinylog.Logger;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class AutonomousConstants {
    /** A horizontal shift on the robot's position relative to reef poles. */
    public static final double DEFAULT_REEF_HORIZONTAL_ALIGNMENT_FUDGE = Units.inchesToMeters(0);
    public static final double FUDGE_INCREMENT = 0.25; // Inches

    // Time in seconds after we start scoring the coral to give up on hitting the setpoint
    public static final double SCORING_TIME = 0.25;

    // Translational Constraints in Meters
    public static final TrapezoidProfile.Constraints TRANSLATION_ALIGNMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 4);
    public static final double TRANSLATION_ALIGNMENT_KP = 5, TRANSLATION_ALIGNMENT_KI = 0, TRANSLATION_ALIGNMENT_KD = 0.25;
    public static final double X_TOLERANCE = Units.inchesToMeters(1.25);
    public static final double Y_TOLERANCE = Units.inchesToMeters(1.25);
    public static final double TRANSLATION_ALIGNMENT_TOLERANCE = 0.025;

    // Rotational Constraints in Radians
    public static final TrapezoidProfile.Constraints ROTATION_ALIGNMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI);
    public static final double ROTATION_ALIGNMENT_KP = 5, ROTATION_ALIGNMENT_KI = 0, ROTATION_ALIGNMENT_KD = 0.5;
    public static final double ROTATION_ALIGNMENT_TOLERANCE = Units.degreesToRadians(0.25);

    /** Distance between the centers of the reef poles on the same side of the reef. */
    private static final double DISTANCE_BETWEEN_REEF_POLES = Units.inchesToMeters(12.94);
    /** Perpendicular distance from the center of the reef to the center of the chassis once aligned. */
    private static final double SCORE_DISTANCE_FROM_CENTER = Units.inchesToMeters(32.75) + ChassisConstants.BUMPER_WIDTH / 2 + Units.inchesToMeters(0.75);
    private static final double APPROACH_DISTANCE_FROM_CENTER = SCORE_DISTANCE_FROM_CENTER + Units.inchesToMeters(18);
    public static final double APPROACH_DISTANCE_BLEND_FACTOR = Units.inchesToMeters(12);
    public static final double AUTO_SCORE_DISTANCE = Units.inchesToMeters(2);

    private static final Translation2d BLUE_REEF_CENTER = new Translation2d(
        Units.inchesToMeters(144 + 93.5 / 2 - 14),
        VisionConstants.FIELD_LAYOUT.getFieldWidth() / 2
    );
    private static final Translation2d RED_REEF_CENTER = new Translation2d(
        VisionConstants.FIELD_LAYOUT.getFieldLength() - Units.inchesToMeters(144 + 93.5 / 2 - 14),
        VisionConstants.FIELD_LAYOUT.getFieldWidth() / 2
    );

    private static Translation2d RED_CORAL_STATION_RIGHT_TARGET, RED_CORAL_STATION_LEFT_TARGET, BLUE_CORAL_STATION_RIGHT_TARGET, BLUE_CORAL_STATION_LEFT_TARGET;
    private static Translation2d RED_CORAL_STATION_RIGHT_WAYPOINT, RED_CORAL_STATION_LEFT_WAYPOINT, BLUE_CORAL_STATION_RIGHT_WAYPOINT, BLUE_CORAL_STATION_LEFT_WAYPOINT;

    public static List<Translation2d> RED_REEF_APPROACH_POLES;
    public static List<Translation2d> RED_REEF_SCORE_POLES;
    private static List<Translation2d> BLUE_REEF_APPROACH_POLES, BLUE_REEF_SCORE_POLES;

    static {
        computeCoralStationWaypoints();
        computePolePositions(DEFAULT_REEF_HORIZONTAL_ALIGNMENT_FUDGE);
    }

    public static void computeCoralStationWaypoints() {
        Translation2d RED_ORIGIN = new Translation2d(VisionConstants.FIELD_LAYOUT.getFieldLength(), VisionConstants.FIELD_LAYOUT.getFieldWidth());

        // Final Target Points
        BLUE_CORAL_STATION_RIGHT_TARGET = new Translation2d(1.9336, 1.2);
        BLUE_CORAL_STATION_LEFT_TARGET = new Translation2d(1.9336, VisionConstants.FIELD_LAYOUT.getFieldWidth() - 1.2);

        RED_CORAL_STATION_LEFT_TARGET = RED_ORIGIN.minus(BLUE_CORAL_STATION_LEFT_TARGET);
        RED_CORAL_STATION_RIGHT_TARGET = RED_ORIGIN.minus(BLUE_CORAL_STATION_RIGHT_TARGET);

        // Approach Target Points
        BLUE_CORAL_STATION_RIGHT_WAYPOINT = new Translation2d(2.7672, 1.35);
        BLUE_CORAL_STATION_LEFT_WAYPOINT = new Translation2d(2.7672, VisionConstants.FIELD_LAYOUT.getFieldWidth() - 1.35);

        RED_CORAL_STATION_LEFT_WAYPOINT = RED_ORIGIN.minus(BLUE_CORAL_STATION_LEFT_WAYPOINT);
        RED_CORAL_STATION_RIGHT_WAYPOINT = RED_ORIGIN.minus(BLUE_CORAL_STATION_RIGHT_WAYPOINT);
    }

    public static void computePolePositions(double fudge) {
        List<Translation2d> APPROACH_REEF_POLES =
            (IntStream.range(0, 12))
                .mapToObj(i -> new Translation2d(
                    APPROACH_DISTANCE_FROM_CENTER, (i % 2 == 0 ? -DISTANCE_BETWEEN_REEF_POLES : DISTANCE_BETWEEN_REEF_POLES) / 2 - fudge
                ).rotateBy(Rotation2d.fromDegrees(60).times(Math.floor((double) i / 2))))
                .toList();

        List<Translation2d> SCORE_REEF_POLES =
            (IntStream.range(0, 12))
                .mapToObj(i -> new Translation2d(
                    SCORE_DISTANCE_FROM_CENTER, (i % 2 == 0 ? -DISTANCE_BETWEEN_REEF_POLES : DISTANCE_BETWEEN_REEF_POLES) / 2 - fudge
                ).rotateBy(Rotation2d.fromDegrees(60).times(Math.floor((double) i / 2))))
                .toList();

        RED_REEF_APPROACH_POLES = APPROACH_REEF_POLES.stream().map(p -> p.plus(RED_REEF_CENTER)).collect(Collectors.toList());
        RED_REEF_SCORE_POLES = SCORE_REEF_POLES.stream().map(p -> p.plus(RED_REEF_CENTER)).collect(Collectors.toList());

        BLUE_REEF_APPROACH_POLES = APPROACH_REEF_POLES.stream().map(p -> p.plus(BLUE_REEF_CENTER)).collect(Collectors.toList());
        BLUE_REEF_SCORE_POLES = SCORE_REEF_POLES.stream().map(p -> p.plus(BLUE_REEF_CENTER)).collect(Collectors.toList());
        Collections.rotate(BLUE_REEF_APPROACH_POLES, 6);
        Collections.rotate(BLUE_REEF_SCORE_POLES, 6);
    }

    public static Objective getNearestReefPoleScorePosition(Translation2d currentTranslation) {
        DriverStation.Alliance alliance = getAlliance();

        var approachPoles = alliance == DriverStation.Alliance.Blue ? BLUE_REEF_APPROACH_POLES : RED_REEF_APPROACH_POLES;

        Translation2d approach = currentTranslation.nearest(approachPoles);
        int index = approachPoles.indexOf(approach);

        return getObjectiveForPole(index);
    }

    public static Objective getObjectiveForPole(int poleIndex) {
        DriverStation.Alliance alliance = getAlliance();

        var approachPoles = alliance == DriverStation.Alliance.Blue ? BLUE_REEF_APPROACH_POLES : RED_REEF_APPROACH_POLES;
        var scorePoles = alliance == DriverStation.Alliance.Blue ? BLUE_REEF_SCORE_POLES : RED_REEF_SCORE_POLES;

        Translation2d approach = approachPoles.get(poleIndex);
        Translation2d score = scorePoles.get(poleIndex);

        Rotation2d angle = Rotation2d.fromDegrees(60).times(Math.floor((double) poleIndex / 2))
                                     .plus(alliance == DriverStation.Alliance.Blue ? Rotation2d.k180deg : Rotation2d.kZero);

        int side = poleIndex / 2;
        int tag = alliance == DriverStation.Alliance.Blue ?
            ((side < 2) ? 18 - side : 22 - (side - 2)) :
            ((side < 5) ? 7 + side : 6);

        return new Objective(tag, new Pose2d(approach, angle), new Pose2d(score, angle));
    }

    public static Objective getObjectiveForCoralStation(boolean isLeft, Translation2d from) {
        DriverStation.Alliance alliance = getAlliance();

        Translation2d targetPosition = alliance == DriverStation.Alliance.Blue ?
            isLeft ? BLUE_CORAL_STATION_LEFT_TARGET : BLUE_CORAL_STATION_RIGHT_TARGET :
            isLeft ? RED_CORAL_STATION_LEFT_TARGET : RED_CORAL_STATION_RIGHT_TARGET;

        Rotation2d angle = targetPosition.minus(from).getAngle();

        return new Objective(
            getObjectiveForPole((isLeft ? 'K' : 'D') - 'A').tag,
            null,
            new Pose2d(targetPosition, angle)
        );
    }

    public static DriverStation.Alliance getAlliance() {
        return DriverStation.getAlliance().orElseGet(() -> {
            Logger.error("Alliance not specified! Defaulting to Blue...");
            return DriverStation.Alliance.Blue;
        });
    }

    public static boolean isCoralInStationArea(Translation2d coralPosition, boolean isLeft) {
        DriverStation.Alliance alliance = getAlliance();
        return
            (alliance == DriverStation.Alliance.Blue ? coralPosition.getX() < 4.5 : coralPosition.getX() > VisionConstants.FIELD_LAYOUT.getFieldLength() - 4.5)
            && (isLeft ? coralPosition.getY() > VisionConstants.FIELD_LAYOUT.getFieldWidth() - 3 : coralPosition.getY() < 3);
    }

    public record Objective(int tag, Pose2d approachPose, Pose2d scorePose) {
        public DriveToPoseV4Command driveToPoseV4Command() {
            return new DriveToPoseV4Command(
                tag(), AutonomousConstants.APPROACH_DISTANCE_BLEND_FACTOR,
                approachPose(),
                scorePose()
            );
        }

        public Objective fudgeY(double distance) {
            return new Objective(
                tag,
                approachPose.transformBy(new Transform2d(0, distance, new Rotation2d())),
                scorePose.transformBy(new Transform2d(0, distance, new Rotation2d()))
            );
        }
    }
}
