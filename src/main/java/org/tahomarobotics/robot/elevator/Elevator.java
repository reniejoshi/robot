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

package org.tahomarobotics.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Elevator {
    private final ElevatorSubsystem elevator;

    public Elevator() {
        this(new ElevatorSubsystem());
    }

    Elevator(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    public Command moveToMinPosition() {
        return Commands.either(
            moveToMinPositionDiscrete(),
            moveToMinPositionContinuous(),
            elevator.discreteMode)
               .andThen(Commands.waitUntil(elevator::isAtTargetPosition));
    }

    public Command moveToMinPositionDiscrete() {
        return elevator.runOnce(elevator::moveToMinPositionDiscrete);
    }

    public Command moveToMinPositionContinuous() {
        return elevator.runOnce(elevator::moveToMinPositionContinuous);
    }

    public Command moveToMaxPosition() {
        return Commands.either(
            moveToMaxPositionDiscrete(),
            moveToMaxPositionContinuous(),
            elevator.discreteMode)
               .andThen(Commands.waitUntil(elevator::isAtTargetPosition));
    }

    public Command moveToMaxPositionDiscrete() {
        return elevator.runOnce(elevator::moveToMaxPositionDiscrete);
    }

    public Command moveToMaxPositionContinuous() {
        return elevator.runOnce(elevator::moveToMaxPositionContinuous);
    }

    public Command toggleMode() {
        return elevator.runOnce(elevator::toggleMode);
    }
}
