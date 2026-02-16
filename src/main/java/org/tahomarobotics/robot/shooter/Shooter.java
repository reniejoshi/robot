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

package org.tahomarobotics.robot.shooter;

import org.tahomarobotics.robot.shooter.ShooterSubsystem.FlywheelMotorState;
import org.tahomarobotics.robot.shooter.ShooterSubsystem.PivotMotorState;
import org.tinylog.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter {
    private final ShooterSubsystem shooter;

    public Shooter() {
        this(new ShooterSubsystem());
        Logger.info("Creating an instance of Shooter....");
    }

    Shooter(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    public Command shoot() {
        return shooter.runOnce(() -> shooter.setPivotMotorState(PivotMotorState.DEPLOYED))
            .andThen(shooter.runOnce(() -> shooter.setFlywheelMotorState(FlywheelMotorState.SHOOTING)));
    }

    public Command stow() {
        return shooter.runOnce(() ->shooter.setFlywheelMotorState(FlywheelMotorState.IDLE))
            .andThen(shooter.runOnce(() -> shooter.setPivotMotorState(PivotMotorState.STOWED)));
    }

    public Trigger isShooting() {
        return new Trigger(() -> shooter.getFlywheelMotorState() == FlywheelMotorState.SHOOTING);
    }
}
