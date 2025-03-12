package org.tahomarobotics.robot.windmill;

import edu.wpi.first.math.Pair;
import org.tahomarobotics.robot.util.motion.MotionProfile;
import org.tahomarobotics.robot.util.motion.MotionState;
import org.tahomarobotics.robot.util.motion.TrapezoidalMotionProfile;
import org.tinylog.Logger;

import java.util.Optional;

public class WindmillTrajectory {

    private static final double ELEV_MAX_VEL = WindmillConstants.ELEVATOR_MAX_VELOCITY;
    private static final double ELEV_MAX_ACC = WindmillConstants.ELEVATOR_MAX_ACCELERATION;
    private static final double ARM_MAX_VEL = WindmillConstants.ARM_MAX_VELOCITY;
    private static final double ARM_MAX_ACC = WindmillConstants.ARM_MAX_ACCELERATION;

    protected record WindmillProfile(MotionProfile elev, MotionProfile arm) {}

    public final String name;
    private final WindmillProfile[] profiles;
    public final WindmillState[] states;

    MotionState e = new MotionState();
    MotionState a = new MotionState();

    public WindmillTrajectory(String name, WindmillState[] states) throws MotionProfile.MotionProfileException {
        this.name = name;
        this.states = states;
        if (states == null || states.length < 2) {
            throw new IllegalArgumentException("states either null or less than two");
        }


        profiles = new WindmillProfile[states.length - 1];
        WindmillProfile prior = null;
        for (int i = 0; i < profiles.length; i++) {
            prior = profiles[i] = createProfile(prior, i, states);

        }
    }


    private WindmillProfile createProfile(WindmillProfile prior, int index, WindmillState[] states) throws MotionProfile.MotionProfileException {

        double startTime = 0;
        double elevStartVelocity = 0;
        double armStartVelocity = 0;
        if (prior != null) {
            startTime = prior.elev.getEndTime();
            elevStartVelocity = prior.elev.getLastMotionState().velocity;
            armStartVelocity = prior.arm.getLastMotionState().velocity;
        }


        // determine ending velocities
        double elevEndVelocity = 0.0;
        double armEndVelocity = 0.0;

        if (states.length > index + 2) {

            double currentDirection = Math.signum(states[index + 1].elevatorState().heightMeters() - states[index].elevatorState().heightMeters());
            double nextDirection = Math.signum(states[index + 2].elevatorState().heightMeters() - states[index + 1].elevatorState().heightMeters());
            if (currentDirection == nextDirection) {
                elevEndVelocity = ELEV_MAX_VEL;
            }

            currentDirection = Math.signum(states[index + 1].armState().angleRadians() - states[index].armState().angleRadians());
            nextDirection = Math.signum(states[index + 2].armState().angleRadians() - states[index + 1].armState().angleRadians());
            if (currentDirection == nextDirection) {
                armEndVelocity = ARM_MAX_VEL;
            }
        }


        MotionProfile elev = new TrapezoidalMotionProfile(
            startTime,
            states[index].elevatorState().heightMeters(),
            states[index + 1].elevatorState().heightMeters(),
            elevStartVelocity, elevEndVelocity,
            ELEV_MAX_VEL, ELEV_MAX_ACC
        );

        MotionProfile arm = new TrapezoidalMotionProfile(
            startTime,
            states[index].armState().angleRadians(),
            states[index + 1].armState().angleRadians(),
            armStartVelocity, armEndVelocity,
            ARM_MAX_VEL, ARM_MAX_ACC
        );

        if (elev.getEndTime() > arm.getEndTime()) {
            arm = arm.updateEndTime(elev.getEndTime());
        } else {
            elev = elev.updateEndTime(arm.getEndTime());
        }

        return new WindmillProfile(elev, arm);
    }

    public WindmillState sample(double time) {
        time = Math.min(Math.max(time, profiles[0].elev.startTime), profiles[profiles.length - 1].elev.getEndTime());

        for (WindmillProfile profile : profiles) {
            if (time <= profile.elev.getEndTime()) {
                profile.elev.getSetpoint(time, e);
                profile.arm.getSetpoint(time, a);
                break;
            }
        }
        return new WindmillState(
            time, new WindmillState.ElevatorState(e.position, e.velocity, e.acceleration),
            new WindmillState.ArmState(a.position, a.velocity, a.acceleration)
        );
    }

    public WindmillState getInitialState() {
        return sample(profiles[0].elev.startTime);
    }

    public double getTotalTimeSeconds() {
        return profiles[profiles.length - 1].elev.getEndTime() - profiles[0].elev.startTime;
    }

    public static Optional<WindmillTrajectory> loadTrajectories(Pair<WindmillConstants.TrajectoryState, WindmillConstants.TrajectoryState> fromTo) {
        Logger.info("Loading trajectory {}", fromTo);


        return Optional.empty();
    }

}
