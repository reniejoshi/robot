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
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.robot.shooter.ShooterConstants.*;

public class ShooterSubsystem extends AbstractSubsystem {
    // -- Motors --
    private final TalonFX pivotMotor = new TalonFX(RobotMap.SHOOTER_PIVOT_MOTOR);
    private final TalonFX flywheelMotor = new TalonFX(RobotMap.SHOOTER_FLYWHEEL_MOTOR);
    private final TalonFX passthroughMotor = new TalonFX(RobotMap.SHOOTER_PASSTHROUGH_MOTOR);

    // -- Pivot motor status signals --
    private final StatusSignal<Angle> pivotMotorPosition;
    private final StatusSignal<AngularVelocity> pivotMotorVelocity;
    private final StatusSignal<Voltage> pivotMotorVoltage;
    private final StatusSignal<Current> pivotMotorCurrent;

    // -- Flywheel motor status signals --
    private final StatusSignal<AngularVelocity> flywheelMotorVelocity;
    private final StatusSignal<Voltage> flywheelMotorVoltage;
    private final StatusSignal<Current> flywheelMotorCurrent;

    // -- Control requests --
    private final VoltageOut voltageControl = new VoltageOut(0);

    // -- States --
    private PivotMotorState pivotMotorState = PivotMotorState.STOWED;
    private FlywheelMotorState flywheelMotorState = FlywheelMotorState.IDLE;

    ShooterSubsystem() {
        // Configure motors
        RobustConfigurator.tryConfigureTalonFX("Pivot Shooter Motor", pivotMotor, createShooterMotorConfig);
        RobustConfigurator.tryConfigureTalonFX("Flywheel Shooter Motor", flywheelMotor, createShooterMotorConfig);

        // Bind pivot motor status signals
        pivotMotorPosition = pivotMotor.getPosition();
        pivotMotorVelocity = pivotMotor.getVelocity();
        pivotMotorVoltage = pivotMotor.getMotorVoltage();
        pivotMotorCurrent = pivotMotor.getSupplyCurrent();

        // Bind flywheel motor status signals
        flywheelMotorVelocity = flywheelMotor.getVelocity();
        flywheelMotorVoltage = flywheelMotor.getMotorVoltage();
        flywheelMotorCurrent = flywheelMotor.getSupplyCurrent();

        zeroPivotMotor();
    }

    @Override
    public void subsystemPeriodic() {
        BaseStatusSignal.refreshAll(pivotMotorPosition, pivotMotorVoltage, pivotMotorCurrent,
            flywheelMotorVelocity, flywheelMotorVoltage, flywheelMotorCurrent);
        
        Logger.recordOutput("Shooter/Pivot Motor/Position", pivotMotorPosition.getValue());
        Logger.recordOutput("Shooter/Pivot Motor/Velocity", pivotMotorVelocity.getValue());
        Logger.recordOutput("Shooter/Pivot Motor/Voltage", pivotMotorVoltage.getValue());
        Logger.recordOutput("Shooter/Pivot Motor/Current", pivotMotorCurrent.getValue());
        Logger.recordOutput("Shooter/Pivot Motor/State", pivotMotorState);
        Logger.recordOutput("Shooter/Pivot Motor/State/Angle", pivotMotorState.angle);
        Logger.recordOutput("Shooter/Pivot Motor/Control Request", pivotMotorState.controlRequest.toString());

        Logger.recordOutput("Shooter/Flywheel Motor/Velocity", flywheelMotorVelocity.getValue());
        Logger.recordOutput("Shooter/Flywheel Motor/Voltage", flywheelMotorVoltage.getValue());
        Logger.recordOutput("Shooter/Flywheel Motor/Current", flywheelMotorCurrent.getValue());
        Logger.recordOutput("Shooter/Flywheel Motor/State", flywheelMotorState);
        Logger.recordOutput("Shooter/Flywheel Motor/State/Velocity", flywheelMotorState.velocity);
        Logger.recordOutput("Shooter/Flywheel Motor/State/Control Request", flywheelMotorState.controlRequest.toString());
    }

    public void zeroPivotMotor() {
        Command zeroCommand = this.runOnce(() -> setPivotMotorVoltage(PIVOT_ZEROING_VOLTAGE))
            .andThen(Commands.waitSeconds(PIVOT_ZEROING_WAIT_SECONDS)
            .andThen(Commands.waitUntil(() -> getPivotMotorVelocity().isNear(PIVOT_ZERO_VELOCITY, PIVOT_ZERO_VELOCITY_THRESHOLD)))
            .andThen(() -> pivotMotor.setPosition(PIVOT_ZERO_ANGLE))
            .andThen(() -> setPivotMotorVoltage(Volts.of(0)))
        );

        RobotModeTriggers.autonomous()
            .or(RobotModeTriggers.teleop())
            .onTrue(zeroCommand);
    }

    // -- Setters --

    public void setPivotMotorState(PivotMotorState pivotMotorState) {
        this.pivotMotorState = pivotMotorState;
        pivotMotor.setControl(pivotMotorState.controlRequest);
    }

    public void setFlywheelMotorState(FlywheelMotorState flywheelMotorState) {
        this.flywheelMotorState = flywheelMotorState;
        flywheelMotor.setControl(flywheelMotorState.controlRequest);
    }

    public void setPivotMotorVoltage(Voltage voltage) {
        pivotMotor.setControl(voltageControl.withOutput(voltage));
    }

    // -- Getters --

    public AngularVelocity getPivotMotorVelocity() {
        return pivotMotorVelocity.getValue();
    }

    // -- States --
    enum PivotMotorState {
        DEPLOYED(new MotionMagicVoltage(PIVOT_DEPLOYED_ANGLE), PIVOT_DEPLOYED_ANGLE),
        STOWED(new MotionMagicVoltage(PIVOT_STOWED_ANGLE), PIVOT_STOWED_ANGLE),
        ZEROING(new VoltageOut(PIVOT_ZEROING_VOLTAGE), PIVOT_ZERO_ANGLE);

        private final ControlRequest controlRequest;
        private final Angle angle;

        PivotMotorState(ControlRequest controlRequest, Angle angle) {
            this.controlRequest = controlRequest;
            this.angle = angle;
        }
    }

    enum FlywheelMotorState {
        SHOOTING(new MotionMagicVelocityVoltage(FLYWHEEL_SHOOTING_VELOCITY), FLYWHEEL_SHOOTING_VELOCITY),
        IDLE(new NeutralOut(), FLYWHEEL_IDLE_VELOCITY);

        private final ControlRequest controlRequest;
        private final AngularVelocity velocity;

        FlywheelMotorState(ControlRequest controlRequest, AngularVelocity velocity) {
            this.controlRequest = controlRequest;
            this.velocity = velocity;
        }
    }
}
