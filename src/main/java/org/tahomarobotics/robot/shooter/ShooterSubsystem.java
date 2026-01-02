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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;

public class ShooterSubsystem extends AbstractSubsystem {
    // Motors
    private final TalonFX pivotMotor;
    private final TalonFX flywheelMotor;
    private final TalonFX passthroughMotor;

    // Control requests
    private final PositionVoltage positionControl = new PositionVoltage(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    // Status signals
    private final StatusSignal<AngularVelocity> flywheelVelocity, passthroughVelocity;

    public ShooterSubsystem() {
        // Initialize hardware
        pivotMotor = new TalonFX(RobotMap.SHOOTER_PIVOT_MOTOR);
        flywheelMotor = new TalonFX(RobotMap.SHOOTER_FLYWHEEL_MOTOR);
        passthroughMotor = new TalonFX(RobotMap.SHOOTER_PASSTHROUGH_MOTOR);

        // Initialize status signals
        flywheelVelocity = flywheelMotor.getVelocity();
        passthroughVelocity = passthroughMotor.getVelocity();

        org.tinylog.Logger.info("Creating an instance of ShooterSubsystem....");
    }

    ShooterSubsystem(TalonFX pivotMotor, TalonFX flywheelMotor, TalonFX passthroughMotor) {
        this.pivotMotor = pivotMotor;
        this.flywheelMotor = flywheelMotor;
        this.passthroughMotor = passthroughMotor;

        // Initialize status signals
        flywheelVelocity = this.flywheelMotor.getVelocity();
        passthroughVelocity = this.passthroughMotor.getVelocity();
    }

    // Setters

    public void setPassthroughIntakingVelocity() {
        passthroughMotor.setControl(velocityControl.withVelocity(ShooterConstants.PASSTHROUGH_VELOCITY));
    }

    public void setFlywheelShootingVelocity() {
        flywheelMotor.setControl(velocityControl.withVelocity(ShooterConstants.FLYWHEEL_VELOCITY));
    }

    // Getters

    public boolean isFlywheelAtShootingVelocity() {
        return flywheelVelocity.getValue().isNear(ShooterConstants.FLYWHEEL_VELOCITY, ShooterConstants.THRESHOLD);
    }

    public boolean isPassthroughAtIntakingVelocity() {
        return passthroughVelocity.getValue().isNear(ShooterConstants.PASSTHROUGH_VELOCITY, ShooterConstants.THRESHOLD);
    }

    @Override
    public void subsystemPeriodic() {
        BaseStatusSignal.refreshAll(flywheelVelocity, passthroughVelocity);
    }
}
