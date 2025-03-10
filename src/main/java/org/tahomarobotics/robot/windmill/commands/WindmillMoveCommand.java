package org.tahomarobotics.robot.windmill.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.windmill.*;
import org.tahomarobotics.robot.windmill.WindmillConstants.TrajectoryState;
import org.tinylog.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class WindmillMoveCommand extends Command {
    private static final boolean DEBUG = false;
    private static final double TIME_ELAPSED_TOLERANCE = 0.05;

    // Subsystems

    private final Windmill windmill = Windmill.getInstance();

    // Trajectory

    private final Pair<TrajectoryState, TrajectoryState> fromTo;
    private final WindmillTrajectory trajectory;

    // State

    private final Timer timer = new Timer();
    List<double[]> data = new ArrayList<>();

    // Command

    private WindmillMoveCommand(Pair<TrajectoryState, TrajectoryState> fromTo, WindmillTrajectory trajectory) {
        Logger.info("Created move command from {} to {}.", fromTo.getFirst(), fromTo.getSecond());

        this.fromTo = fromTo;
        this.trajectory = trajectory;

        addRequirements(windmill);
    }

    @Override
    public void initialize() {
        // TODO: why are we checking for at target state
        if (!windmill.isAtTargetTrajectoryState()) {
            Logger.error(
                "Windmill was not within tolerance for starting state! Arm was at ({}, {}) but needs to be at ({}, {})", windmill.getElevatorHeight(),
                windmill.getArmPosition(), fromTo.getFirst().elev, fromTo.getFirst().arm );
            cancel();
            return;
        }

        windmill.setTargetState(fromTo.getSecond());

        Logger.info("Running trajectory: '{}'", trajectory.name);
        timer.restart();
    }

    @Override
    public void execute() {
        double time = timer.get();
        WindmillState state = trajectory.sample(time);
        windmill.setState(state);

        var current = windmill.getCurrentState();

        data.add(new double[] {
            time,
            state.elevatorState().heightMeters(),
            state.elevatorState().velocityMetersPerSecond(),
            state.armState().angleRadians(),
            state.armState().velocityRadiansPerSecond(),

            current.elevatorState().heightMeters(),
            current.elevatorState().velocityMetersPerSecond(),
            current.armState().angleRadians(),
            current.armState().velocityRadiansPerSecond()
        });

        if (DEBUG) {
            Logger.info(
                """
                Endpoint ({0.0} seconds): State: ({+0.000;-0.000} meters, {+0.000;-0.000} degrees)
                """.trim(), time, state.elevatorState().heightMeters(),
                Units.radiansToDegrees(state.armState().angleRadians())
            );
        }
    }

    @Override
    public boolean isFinished() {
        return windmill.isAtTargetTrajectoryState() || timer.hasElapsed(trajectory.getTotalTimeSeconds() + TIME_ELAPSED_TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        if (!data.isEmpty()) {
            double raw[] = new double[data.size() * data.get(0).length];
            int i = 0;
            for (var d : data) {
                for (var r : d) {
                    raw[i++] = r;
                }
            }
            SmartDashboard.putNumberArray("WindmillMoveCommand", raw);
        }
    }

    @Override
    public String getName() {
        return "Windmill Move Command";
    }

    // -- Helpers --

    public static Optional<Command> fromTo(TrajectoryState from, TrajectoryState to) {
        return WindmillTrajectories.getTrajectory(from, to).map(t -> new WindmillMoveCommand(Pair.of(from, to), t));
    }
}
