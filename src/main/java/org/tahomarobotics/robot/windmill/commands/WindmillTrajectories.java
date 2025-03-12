package org.tahomarobotics.robot.windmill.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.windmill.WindmillState;
import org.tahomarobotics.robot.windmill.WindmillTrajectory;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import static org.tahomarobotics.robot.windmill.WindmillConstants.*;

public class WindmillTrajectories {

    private static final Map<Pair<TrajectoryState, TrajectoryState>, WindmillTrajectory> trajectories = new HashMap<>();

    public static Optional<WindmillTrajectory> getTrajectory(TrajectoryState from, TrajectoryState to) {
        WindmillTrajectory traj = trajectories.get(Pair.of(from, to));
        return Optional.ofNullable(traj);
    }

    static {
        WindmillState collectLift = createWindmillState(Units.inchesToMeters(25), Units.degreesToRadians(267.809));
        // Algae
        create(TrajectoryState.COLLECT, TrajectoryState.HIGH_DESCORE, new WindmillState[]{collectLift});
        create(TrajectoryState.COLLECT, TrajectoryState.LOW_DESCORE, new WindmillState[]{collectLift});
        create(TrajectoryState.HIGH_DESCORE, TrajectoryState.COLLECT, new WindmillState[]{collectLift});
        create(TrajectoryState.LOW_DESCORE, TrajectoryState.COLLECT, new WindmillState[]{collectLift});
        create(TrajectoryState.HIGH_DESCORE, SMALL_PULLBACK, TrajectoryState.LOW_DESCORE);
        create(TrajectoryState.LOW_DESCORE, SMALL_PULLBACK, TrajectoryState.HIGH_DESCORE);

        // Algae - Coral
        create(TrajectoryState.L2, SMALL_PULLBACK, TrajectoryState.HIGH_DESCORE);
        create(TrajectoryState.L2, TrajectoryState.LOW_DESCORE);
        create(TrajectoryState.L3, TrajectoryState.HIGH_DESCORE);
        create(TrajectoryState.L3, STANDARD_PULLBACK, TrajectoryState.LOW_DESCORE);
        create(TrajectoryState.L4, LARGE_PULLBACK, TrajectoryState.HIGH_DESCORE);
        create(TrajectoryState.L4, LARGE_PULLBACK, TrajectoryState.LOW_DESCORE);

        // Coral
        create(TrajectoryState.COLLECT, TrajectoryState.L2, new WindmillState[]{collectLift});
        create(TrajectoryState.L2, TrajectoryState.COLLECT, new WindmillState[]{collectLift});
        create(TrajectoryState.STOW, TrajectoryState.COLLECT, new WindmillState[]{collectLift});
        create(TrajectoryState.COLLECT, TrajectoryState.STOW, new WindmillState[]{collectLift});

        create(TrajectoryState.COLLECT, TrajectoryState.L3);
        create(TrajectoryState.L3, TrajectoryState.COLLECT);

        create(TrajectoryState.COLLECT, TrajectoryState.L4);
        create(TrajectoryState.L4, TrajectoryState.COLLECT);

        create(TrajectoryState.L2, LARGE_PULLBACK, TrajectoryState.L3);
        create(TrajectoryState.L2, LARGE_PULLBACK, TrajectoryState.L4);
        create(TrajectoryState.L3, LARGE_PULLBACK, TrajectoryState.L2);
        create(TrajectoryState.L3, LARGE_PULLBACK, TrajectoryState.L4);
        create(TrajectoryState.L4, LARGE_PULLBACK, TrajectoryState.L2);
        create(TrajectoryState.L4, LARGE_PULLBACK, TrajectoryState.L3);

        create(TrajectoryState.START, TrajectoryState.L4);

        create(TrajectoryState.STOW, TrajectoryState.L2);
        create(TrajectoryState.STOW, TrajectoryState.L3);
        create(TrajectoryState.STOW, TrajectoryState.L4);

        create(TrajectoryState.L2, TrajectoryState.STOW);
        create(TrajectoryState.L3, TrajectoryState.STOW);
        create(TrajectoryState.L4, TrajectoryState.STOW);
    }

    private static WindmillState createWindmillState(double elev, double arm) {
        return new WindmillState(
            0,
            new WindmillState.ElevatorState(elev, 0, 0),
            new WindmillState.ArmState(arm, 0, 0)
        );
    }

    private static void create(TrajectoryState start, double pullback, TrajectoryState end) {
        double elev = (start.elev + end.elev) / 2;
        double arm = (start.arm + end.arm) / 2 - pullback;
        var pos = new WindmillState(
            0,
            new WindmillState.ElevatorState(elev, 0, 0),
            new WindmillState.ArmState(arm, 0, 0)
        );
        create(start, end, new WindmillState[]{pos});
    }

    private static void create(TrajectoryState start, TrajectoryState end, WindmillState midPositions[]) {
        WindmillState[] states = new WindmillState[2 + midPositions.length];
        states[0] = start.state;
        states[states.length - 1] = end.state;
        for (int i = 0; i < midPositions.length; i++) {
            states[i + 1] = midPositions[i];
        }
        String name = start.name() + "_TO_" + end.name();
        try {
            trajectories.put(new Pair<>(start, end), new WindmillTrajectory(name, states));
        } catch (Exception e) {
            System.err.println("Trajectory not found for " + name);
        }
    }

    private static void create(TrajectoryState... trajectoryStates) {
        TrajectoryState start = trajectoryStates[0];
        TrajectoryState end = trajectoryStates[trajectoryStates.length - 1];
        WindmillState[] states = new WindmillState[trajectoryStates.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = trajectoryStates[i].state;
        }
        String name = start.name() + "_TO_" + end.name();
        try {
            trajectories.put(new Pair<>(start, end), new WindmillTrajectory(name, states));
        } catch (Exception e) {
            System.err.println("Trajectory not found for " + name);
        }
    }

    public static void initialize() {}
}
