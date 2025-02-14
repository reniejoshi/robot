package org.tahomarobotics.robot.windmill.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tinylog.Logger;

public class WindmillCommands {
    public static Command createCalibrateElevatorCommand(Windmill subsystem) {
        String FINALIZE_KEY = "Finalize";

        Command cmd = (
            Commands.waitUntil(() -> SmartDashboard.getBoolean(FINALIZE_KEY, false))
                    .beforeStarting(() -> {
                        SmartDashboard.putBoolean(FINALIZE_KEY, false);
                        subsystem.initializeElevatorCalibration();
                        Logger.info("Calibrating elevator...");
                    }).finallyDo(interrupted -> {
                        if (interrupted) {
                            Logger.info("Cancelling elevator calibration.");
                            subsystem.applyElevatorOffset();
                        } else {
                            Logger.info("Elevator calibrated!");
                            subsystem.finalizeElevatorCalibration();
                        }
                    })
        ).onlyWhile(RobotState::isDisabled).ignoringDisable(true);
        cmd.addRequirements(subsystem);

        return cmd;
    }

    public static Command createCalibrateArmCommand(Windmill subsystem) {
        String FINALIZE_KEY = "Finalize";

        Command cmd = (
            Commands.waitUntil(() -> SmartDashboard.getBoolean(FINALIZE_KEY, false))
                    .beforeStarting(() -> {
                        SmartDashboard.putBoolean(FINALIZE_KEY, false);
                        subsystem.initializeArmCalibration();
                        Logger.info("Calibrating arm...");
                    }).finallyDo(interrupted -> {
                        if (interrupted) {
                            Logger.info("Cancelling arm calibration.");
                            subsystem.applyArmOffset();
                        } else {
                            Logger.info("Arm calibrated!");
                            subsystem.finalizeArmCalibration();
                        }
                    })
        ).onlyWhile(RobotState::isDisabled).ignoringDisable(true);
        cmd.addRequirements(subsystem);

        return cmd;
    }
}
