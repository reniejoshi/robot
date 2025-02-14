package org.tahomarobotics.robot.windmill.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.WindmillKinematics;
import org.tahomarobotics.robot.windmill.WindmillState;
import org.tahomarobotics.robot.windmill.WindmillTrajectory;
import org.tinylog.Logger;

import java.util.List;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class WindmillMoveCommand extends Command {
    private final Windmill windmill = Windmill.getInstance();

    private final WindmillTrajectory windmillTrajectory;
    private final Timer timer = new Timer();

    public WindmillMoveCommand(WindmillTrajectory windmillTrajectory) {
        this.windmillTrajectory = windmillTrajectory;

        addRequirements(windmill);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        double time = timer.get();
        try {
            WindmillState state = windmillTrajectory.sample(time);
            windmill.setState(state);

            Pose2d sample = windmillTrajectory.samplePose2d(time);
            Logger.info(
                """
                Endpoint ({0.0} seconds): ({+0.000;-0.000} meters, {+0.000;-0.000} meters) -> State: ({+0.000;-0.000} meters, {+0.000;-0.000} degrees)
                """.trim(),
                time,
                sample.getX(), sample.getY(),
                state.elevatorState().heightMeters(), Units.radiansToDegrees(state.armState().angleRadians())
            );
        } catch (WindmillKinematics.KinematicsException e) {
            Logger.error("Kinematics Error: {}", e);
            cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(windmillTrajectory.getTotalTimeSeconds());
    }
}
