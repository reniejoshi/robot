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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.arm.Arm;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.diffyarm.DiffyArm;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.shooter.Shooter;

public class OI {
    // Subsystems
    private final Arm arm;
    private final Collector collector;
    private final DiffyArm diffyArm;
    private final Elevator elevator;
    private final Shooter shooter;

    // -- Constants --

    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRANSLATIONAL_SENSITIVITY = 1.3;

    private static final double DEADBAND = 0.09;
    private static final double TRIGGER_DEADBAND = 0.05;

    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController lessImportantController = new CommandXboxController(1);

    public OI(RobotContainer robotContainer) {
        DriverStation.silenceJoystickConnectionWarning(true);

        this.arm = robotContainer.arm;
        this.collector = robotContainer.collector;
        this.diffyArm = robotContainer.diffyArm;
        this.elevator = robotContainer.elevator;
        this.shooter = robotContainer.shooter;

        configureControllerBindings();
        configureLessImportantControllerBindings();

        setDefaultCommands();
    }

    // -- Bindings --

    public void configureControllerBindings() {
        // Up moves arm clockwise, down moves arm counterclockwise
        arm.setDefaultCommand(arm.setArmPosition(this::getRightY));

        // Moves wrist clockwise
        controller.rightTrigger().whileTrue(arm.setWristPositionClockwise());

        // Moves wrist counterclockwise
        controller.leftTrigger().whileTrue(arm.setWristPositionCounterclockwise());

        // Moves elevator to top
        controller.y().onTrue(elevator.moveToTopPosition());

        // Moves elevator to bottom
        controller.a().onTrue(elevator.moveToBottomPosition());

        // Toggle continuous vs discrete mode
        controller.b().onTrue(elevator.toggleMode());

        // x-axis controls elbow position, y-axis controls wrist position
        // Up moves diffy arm joint clockwise, down moves diffy arm joint counterclockwise
        diffyArm.setDefaultCommand(diffyArm.setArmPosition(this::getLeftX, this::getLeftY));
    }

    public void configureLessImportantControllerBindings() {
    }

    @SuppressWarnings("SuspiciousNameCombination")
    public void setDefaultCommands() {
    }

    // -- Inputs --

    public double getLeftX() {
        return -desensitizePowerBased(controller.getLeftX(), TRANSLATIONAL_SENSITIVITY);
    }

    public double getLeftY() {
        return -desensitizePowerBased(controller.getLeftY(), TRANSLATIONAL_SENSITIVITY);
    }

    public double getRightX() {
        return -desensitizePowerBased(controller.getRightX(), ROTATIONAL_SENSITIVITY);
    }

    public double getRightY() {
        return -desensitizeDeadbandBased(controller.getRightY());
    }

    // -- Helper Methods --

    public double desensitizePowerBased(double value, double power) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }

    public double desensitizeDeadbandBased(double value) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        return value;
    }
}
