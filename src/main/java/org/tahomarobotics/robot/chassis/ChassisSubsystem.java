/*
 * Copyright 2026 Tahoma Robotics
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

package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.Degrees;
import static org.tahomarobotics.robot.RobotMap.*;
import static org.tahomarobotics.robot.RobotMap.BACK_RIGHT_MODULE;

public class ChassisSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    public ChassisSubsystem() {
        this(TalonFX::new, TalonFX::new, CANcoder::new, ChassisConstants.DRIVETRAIN_CONSTANTS,
             ChassisConstants.getModuleConfig(FRONT_LEFT_MODULE, Degrees.of(195.64)),
             ChassisConstants.getModuleConfig(FRONT_RIGHT_MODULE, Degrees.of(-14.94)),
             ChassisConstants.getModuleConfig(BACK_LEFT_MODULE, Degrees.of(-19.07)),
             ChassisConstants.getModuleConfig(BACK_RIGHT_MODULE, Degrees.of(-141.42)));
    }

    ChassisSubsystem(DeviceConstructor<TalonFX> driveMotorConstructor,
                     DeviceConstructor<TalonFX> steerMotorConstructor,
                     DeviceConstructor<CANcoder> encoderConstructor,
                     SwerveDrivetrainConstants drivetrainConstants,
                     SwerveModuleConstants<?, ?, ?>... modules) {
        super(driveMotorConstructor, steerMotorConstructor, encoderConstructor, drivetrainConstants, modules);
    }
}
