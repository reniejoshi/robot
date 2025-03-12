package org.tahomarobotics.robot.auto.autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.auto.AutonomousConstants;
import org.tahomarobotics.robot.auto.commands.DriveToPoseV4Command;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorCommands;
import org.tahomarobotics.robot.grabber.Grabber;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.commands.WindmillCommands;
import org.tahomarobotics.robot.windmill.commands.WindmillMoveCommand;
import org.tinylog.Logger;

import java.util.Set;
import java.util.function.DoubleSupplier;

import static org.tahomarobotics.robot.windmill.WindmillConstants.TrajectoryState.*;

public class FivePiece extends SequentialCommandGroup {
    // -- Constants --

    private static final double SCORING_DISTANCE = Units.inchesToMeters(3);
    private static final double ARM_UP_DISTANCE = AutonomousConstants.APPROACH_DISTANCE_BLEND_FACTOR + Units.inchesToMeters(6);
    private static final double SCORING_TIME = 0.25;
    private static final double COLLECT_TIMEOUT = 0.75;

    // -- Requirements --

    private final Chassis chassis = Chassis.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Grabber grabber = Grabber.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Windmill windmill = Windmill.getInstance();

    // -- Determinants --

    private final DriverStation.Alliance alliance = AutonomousConstants.getAlliance();

    // -- Initialization --

    public FivePiece(boolean isLeft) {
        setName("Five-Piece " + (isLeft ? "Left" : "Right"));

        Timer timer = new Timer();
        addCommands(
            Commands.runOnce(timer::restart),
            Commands.parallel(
                // Assuming proper starting state, this will take one cycle.
                WindmillCommands.createElevatorZeroCommand(Windmill.getInstance()),
                CollectorCommands.createZeroCommand(Collector.getInstance())
            ),
            // Drive to our first scoring position then score
            driveToPoleThenScore(isLeft ? 'J' : 'E', () -> alliance == DriverStation.Alliance.Red && !isLeft ? Units.inchesToMeters(2) : 0),
            driveToCoralStationAndCollect(isLeft),
            // Drive to the second scoring position then score
            driveToPoleThenScoreWhileCollecting(isLeft ? 'K' : 'D'),
            driveToCoralStationAndCollect(isLeft),
            // Drive to the third scoring position then score
            driveToPoleThenScoreWhileCollecting(isLeft ? 'L' : 'C'),
            driveToCoralStationAndCollect(isLeft),
            // Drive to the fourth scoring position then score
            driveToPoleThenScoreWhileCollecting(isLeft ? 'A' : 'B'),
            Commands.runOnce(() -> Logger.info("Five-Piece completed in {} seconds.", timer.get()))
        );
    }

    public Command driveToPoleThenScore(char pole, DoubleSupplier fudge) {
        // Drive to the scoring position
        DriveToPoseV4Command dtp = AutonomousConstants.getObjectiveForPole(pole - 'A').fudgeY(fudge.getAsDouble()).driveToPoseV4Command();

        // Move arm from STOW to L4
        Command stowToL4 = WindmillMoveCommand.fromTo(STOW, L4).orElseThrow();

        // Score grabber
        Command scoreGrabber = grabber.runOnce(grabber::transitionToScoring).andThen(Commands.waitSeconds(SCORING_TIME)).andThen(grabber::transitionToDisabled);

        Timer timer = new Timer();
        return Commands.parallel(
            Commands.runOnce(timer::restart),
            dtp,
            dtp.runWhen(() -> dtp.getTargetWaypoint() == 0 && dtp.getDistanceToWaypoint() <= ARM_UP_DISTANCE, stowToL4),
            dtp.runWhen(
                () -> dtp.getTargetWaypoint() == 1 && dtp.getDistanceToWaypoint() <= SCORING_DISTANCE && windmill.isAtTargetTrajectoryState(),
                scoreGrabber.andThen(Commands.runOnce(() -> Logger.info("Scored grabber.")))
            )
        ).andThen(Commands.runOnce(() -> Logger.info("Driving to {} and scoring took {} seconds.", pole, timer.get())));
    }

    public Command driveToPoleThenScoreWhileCollecting(char pole) {

        // Drive to the scoring position
        DriveToPoseV4Command dtp = AutonomousConstants.getObjectiveForPole(pole - 'A').driveToPoseV4Command();

        // Move arm from STOW to L4
        Command stowToL4 = WindmillMoveCommand.fromTo(STOW, L4).orElseThrow();

        // Score grabber
        Command scoreGrabber = grabber.runOnce(grabber::transitionToScoring).andThen(Commands.waitSeconds(SCORING_TIME)).andThen(grabber::transitionToDisabled);

        Timer timer = new Timer();
        return Commands.race(
            // Run the timer until the commands are done
            Commands.startRun(timer::restart, () -> {}),
            // Drive to the scoring position
            dtp,
            Commands
                // Ensure we are collected by a specific timeout
                .waitUntil(grabber::isHolding)
                // Move the arm to stow once collected
                .andThen(WindmillMoveCommand.fromTo(COLLECT, STOW).orElseThrow())
                // TODO: Possibly disable the indexer and collector?
                .withTimeout(COLLECT_TIMEOUT)
                .andThen(Commands.parallel(
                    // Score the coral if collected
                    dtp.runWhen(() -> dtp.getTargetWaypoint() == 0 && dtp.getDistanceToWaypoint() <= ARM_UP_DISTANCE, stowToL4),
                    dtp.runWhen(() -> dtp.getTargetWaypoint() == 1 && dtp.getDistanceToWaypoint() <= SCORING_DISTANCE, scoreGrabber),
                    // Stay driving if collected, otherwise this command will finish and cancel the DTP command.
                    Commands.idle().onlyWhile(grabber::isHolding)
                ))
        ).andThen(Commands.runOnce(() -> Logger.info("Driving to {} and scoring took {} seconds.", pole, timer.get())));
    }

    private Command driveToCoralStationAndCollect(boolean isLeft) {
        Timer timer = new Timer();
        return Commands.parallel(
            Commands.runOnce(timer::restart),
            // Drive to the coral station using the current translation of the chassis
            Commands.defer(
                () -> AutonomousConstants.getObjectiveForCoralStation(isLeft, Chassis.getInstance().getPose().getTranslation()).driveToPoseV4Command(),
                Set.of(chassis)
            ),
            // Move the arm to STOW then COLLECT, to avoid hitting the arm on the reef.
            Commands.waitSeconds(0.5)
                    .andThen(WindmillMoveCommand.fromTo(L4, COLLECT).orElseThrow())
                    .andThen(grabber.runOnce(grabber::transitionToCollecting)),
            // Collect from the collector and indexer
            collector.runOnce(collector::deploymentTransitionToCollect).andThen(collector.runOnce(collector::collectorTransitionToCollecting)),
            indexer.runOnce(indexer::transitionToCollecting)
        ).andThen(Commands.runOnce(() -> Logger.info("Driving to coral station took {} seconds.", timer.get())));
    }

    // -- Coral Detection Snippet --

    //                .onlyWhile(() -> Vision.getInstance().getCoralPosition().isEmpty()),
    //            Commands.deferredProxy(() -> {
    //                // Get the supposed position of the coral
    //                Translation2d coral = null;
    //                for (int i = 0; i < CORAL_DETECTION_RETRIES; i++) {
    //                    var coralOpt = Vision.getInstance().getCoralPosition();
    //                    if (coralOpt.isPresent() && !AutonomousConstants.isCoralInStationArea(coralOpt.get(), isLeft)) {
    //                        coral = coralOpt.get();
    //                        break;
    //                    }
    //                    Logger.error("[{}/{}] Could not find coral!", i, CORAL_DETECTION_RETRIES);
    //                }
    //
    //                if (coral == null) {
    //                    // TODO: What do we do here...
    //                    return Commands.none().withName("No Coral Found");
    //                }
    //
    //                // Get angle between the chassis and the coral
    //                Rotation2d angle = coral.minus(Chassis.getInstance().getPose().getTranslation()).getAngle();
    //
    //                return new DriveToPoseV4Command(objective.tag(), 0, new Pose2d(coral, angle)).withName("Drive to Coral");
    //            }).onlyWhile(() -> !grabber.isHolding())
}
