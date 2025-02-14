package org.tahomarobotics.robot.windmill;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.tinylog.Logger;

import java.util.List;

import static org.tahomarobotics.robot.windmill.WindmillConstants.*;

public class WindmillKinematicsTests {
//    public static final WindmillTrajectory windmillTrajectory = new WindmillTrajectory(
//        new Pose2d(0, ELEVATOR_LOW_POSE + ARM_LENGTH, Rotation2d.kZero),
//        List.of(
//            new Translation2d(-0.402, 0.752),
//            new Translation2d(-0.659, 0.602),
//            new Translation2d(-0.642, 0.417),
//            new Translation2d(-0.523, 0.306),
//            new Translation2d(-0.267, 0.272)
//        ),
//        new Pose2d(0, 0.219, Rotation2d.kZero),
//        new TrajectoryConfig(0.25, 0.25)
//    );
//
//    @Test
//    public void testKinematics() throws WindmillKinematics.KinematicsException {
//        WindmillState state = null;
//        for (double t = 0; t < windmillTrajectory.getTotalTimeSeconds(); t += 0.1) {
//            Translation2d sample = windmillTrajectory.samplePose2d(t).getTranslation();
//
//            // Using bad trajectory above, so it was clamped to prevent invalid input data
//            double x = MathUtil.clamp(sample.getX(), -ARM_LENGTH, ARM_LENGTH);
//
//            state = windmillTrajectory.sample(t);
//
//            Logger.debug(
//                """
//                    Endpoint ({0.0} seconds): ({+0.000;-0.000} meters, {+0.000;-0.000} meters) -> State: ({+0.000;-0.000} meters, {+0.000;-0.000} degrees)
//                    """.trim(),
//                t,
//                x, sample.getY(),
//                state.elevatorState().heightMeters(), Units.radiansToDegrees(state.armState().angleRadians())
//            );
//
//            Translation2d forward = WindmillKinematics.forwardKinematics(state);
//
//
//            Assertions.assertEquals(x, forward.getX(), 1e-4);
//            Assertions.assertEquals(sample.getY(), forward.getY(), 1e-4);
//        }
//    }
}
