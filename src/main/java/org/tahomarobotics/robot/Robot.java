/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.tinylog.Logger;

public class Robot extends LoggedRobot {

    private final RobotContainer robotContainer;

    // Robot

    private Robot() {
        this(new RobotContainer());
    }

    public Robot(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        // Disable watchdogs
        disableWatchdog(this, IterativeRobotBase.class);
        disableWatchdog(CommandScheduler.getInstance(), CommandScheduler.class);

        // Log various aspects of our robot
        logCommandScheduler();
        // TODO: Possibly need a warmup command for record logging.
        configureAdvantageKit();

        // Simulate Helper Commands
        SmartDashboard.putData(
            "Enable Autonomous Simulation", Commands.runOnce(() -> {
                DriverStationSim.setAutonomous(true);
                DriverStationSim.setEnabled(true);
                DriverStationSim.setAllianceStationId(AllianceStationID.Red1);

                DriverStationSim.notifyNewData();
            }).ignoringDisable(true)
        );
        SmartDashboard.putData(
            "Disable Simulation", Commands.runOnce(() -> {
                DriverStationSim.setEnabled(false);
                DriverStationSim.notifyNewData();
            }).ignoringDisable(true)
        );

        Logger.info("--- Robot Initialized ---");
    }

    private void logCommandScheduler() {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();
        commandScheduler.onCommandInitialize(command -> {
            if (RobotState.isAutonomous()) { Logger.info(command.getName() + " initialized!"); }
        });
        commandScheduler.onCommandFinish(command -> {
            if (RobotState.isAutonomous()) { Logger.info(command.getName() + " finished!"); }
        });
        commandScheduler.onCommandInterrupt((command, by) -> {
            if (RobotState.isAutonomous()) { Logger.warn(command.getName() + " interrupted by {}!", by.map(Command::getName).orElse("Nothing")); }
        });
    }

    private void configureAdvantageKit() {
        org.littletonrobotics.junction.Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        // TODO: If bandwidth usage is too high, add toggle for NT4 logging.
        org.littletonrobotics.junction.Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

        org.littletonrobotics.junction.Logger.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    // Disabled

    @Override
    public void disabledInit() {
        Logger.warn("--- Disabled ---");
    }

    @Override
    public void disabledPeriodic() {}

    // Autonomous

    @Override
    public void autonomousInit() {
        Logger.info("--- Autonomous Initialized ---");
    }

    @Override
    public void autonomousPeriodic() {}

    // Teleop

    @Override
    public void teleopInit() {
        Logger.info("--- TeleOp Initialized ---");
    }

    @Override
    public void teleopPeriodic() {}

    // Test

    @Override
    public void testInit() {
        Logger.info("--- Test Initialized ---");
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {
    }

    // Simulation

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {}

    // Helpers

    private <T> void disableWatchdog(T inst, Class<?> clazz) {
        try {
            var field = clazz.getDeclaredField("m_watchdog");
            field.setAccessible(true);
            field.set(
                inst, new Watchdog(0, () -> {}) {
                    @Override
                    public void addEpoch(String epochName) {}

                    @Override
                    public void printEpochs() {}

                    @Override
                    public void enable() {}

                    @Override
                    public void disable() {}
                }
            );
            Logger.warn("Disabled " + inst.getClass() + "'s watchdog!");
        } catch (NoSuchFieldException | IllegalAccessException ignored) {}
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
