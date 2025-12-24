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

package org.tahomarobotics.robot.diffyarm;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tinylog.Logger;

public class DiffyArmSubsystem extends AbstractSubsystem {
    // Motors
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    public DiffyArmSubsystem() {
        // Initialize hardware
        topMotor = new TalonFX(RobotMap.DIFFY_ARM_TOP_MOTOR);
        bottomMotor = new TalonFX(RobotMap.DIFFY_ARM_BOTTOM_MOTOR);

        Logger.info("Creating an instance of DiffyArmSubsystem...");
    }

    DiffyArmSubsystem(TalonFX topMotor, TalonFX bottomMotor) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
    }

    @Override
    public void subsystemPeriodic() {

    }
}
