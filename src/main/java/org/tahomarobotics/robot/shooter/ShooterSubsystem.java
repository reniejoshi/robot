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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;

import static org.tahomarobotics.robot.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends AbstractSubsystem {
    // Motors
    private final TalonFX pivotMotor = new TalonFX(RobotMap.SHOOTER_PIVOT_MOTOR);
    private final TalonFX flywheelMotor = new TalonFX(RobotMap.SHOOTER_FLYWHEEL_MOTOR);
    private final TalonFX passthroughMotor = new TalonFX(RobotMap.SHOOTER_PASSTHROUGH_MOTOR);

    // Pivot motor status signals
    private final StatusSignal<Angle> pivotMotorPosition;
    private final StatusSignal<Voltage> pivotMotorVoltage;
    private final StatusSignal<Current> pivotMotorCurrent;

    // Flywheel motor status signals
    private final StatusSignal<AngularVelocity> flywheelMotorVelocity;
    private final StatusSignal<Voltage> flywheelMotorVoltage;
    private final StatusSignal<Current> flywheelMotorCurrent;

    ShooterSubsystem() {
        // Configure motors
        RobustConfigurator.tryConfigureTalonFX("Pivot Shooter Motor", pivotMotor, createShooterMotorConfig);
        RobustConfigurator.tryConfigureTalonFX("Flywheel Shooter Motor", flywheelMotor, createShooterMotorConfig);

        // Bind pivot motor status signals
        pivotMotorPosition = pivotMotor.getPosition();
        pivotMotorVoltage = pivotMotor.getMotorVoltage();
        pivotMotorCurrent = pivotMotor.getSupplyCurrent();

        // Bind flywheel motor status signals
        flywheelMotorVelocity = flywheelMotor.getVelocity();
        flywheelMotorVoltage = flywheelMotor.getMotorVoltage();
        flywheelMotorCurrent = flywheelMotor.getSupplyCurrent();
    }

    @Override
    public void subsystemPeriodic() {
        BaseStatusSignal.refreshAll(pivotMotorPosition, pivotMotorVoltage, pivotMotorCurrent,
            flywheelMotorVelocity, flywheelMotorVoltage, flywheelMotorCurrent);
        
        Logger.recordOutput("Shooter/Pivot Motor/Position", pivotMotorPosition.getValue());
        Logger.recordOutput("Shooter/Pivot Motor/Voltage", pivotMotorVoltage.getValue());
        Logger.recordOutput("Shooter/Pivot Motor/Current", pivotMotorCurrent.getValue());

        Logger.recordOutput("Shooter/Flywheel Motor/Velocity", flywheelMotorVelocity.getValue());
        Logger.recordOutput("Shooter/Flywheel Motor/Voltage", flywheelMotorVoltage.getValue());
        Logger.recordOutput("Shooter/Flywheel Motor/Current", flywheelMotorCurrent.getValue());
    }
}
