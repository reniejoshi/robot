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

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class ElevatorSubsystem extends AbstractSubsystem {
    // Motors
    private final TalonFX leftMotor = new TalonFX(RobotMap.ELEVATOR_LEFT_MOTOR);
    private final TalonFX rightMotor = new TalonFX(RobotMap.ELEVATOR_RIGHT_MOTOR);

    // Control requests
    private final PositionVoltage positionControl = new PositionVoltage(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    // Status signals
    private final StatusSignal<Angle> leftMotorPosition = leftMotor.getPosition();

    private boolean isDiscreteMode = false;
    public BooleanSupplier discreteMode = () -> isDiscreteMode;

    private double targetPosition = 0;

    ElevatorSubsystem() {
        rightMotor.setControl(new Follower(RobotMap.ELEVATOR_LEFT_MOTOR, true));
    }

    // Setters
    public void moveToBottomPositionDiscrete() {
        org.tinylog.Logger.info("Move elevator to min position in discrete mode");
        targetPosition = ElevatorConstants.ELEVATOR_BOTTOM_POSITION;
        leftMotor.setControl(positionControl.withPosition(targetPosition));
    }

    public void moveToBottomPositionContinuous() {
        org.tinylog.Logger.info("Move elevator to min position in continuous mode");
        targetPosition = ElevatorConstants.ELEVATOR_BOTTOM_POSITION;
        leftMotor.setControl(velocityControl.withVelocity(RotationsPerSecond.of(-ElevatorConstants.ELEVATOR_RPS)));
    }

    public void moveToTopPositionDiscrete() {
        org.tinylog.Logger.info("Move elevator to max position in discrete mode");
        targetPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;
        leftMotor.setControl(positionControl.withPosition(targetPosition));
    }

    public void moveToTopPositionContinuous() {
        org.tinylog.Logger.info("Move elevator to max position in continuous mode");
        targetPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;
        leftMotor.setControl(velocityControl.withVelocity(RotationsPerSecond.of(ElevatorConstants.ELEVATOR_RPS)));
    }

    public void stopElevatorMotors() {
        org.tinylog.Logger.info("Stop elevator motors");
        leftMotor.setControl(velocityControl.withVelocity(0));
    }

    public void toggleMode() {
        isDiscreteMode = !isDiscreteMode;
    }

    public boolean isAtTargetPosition() {
        return leftMotorPosition.getValue().isNear(Meters.of(targetPosition), ElevatorConstants.THRESHOLD);
    }

    @Override
    public void subsystemPeriodic() {
        StatusSignal.refreshAll(leftMotorPosition);

        Logger.recordOutput("Elevator/Discrete Mode", discreteMode);
        Logger.recordOutput("Elevator/Target Position", targetPosition);
        Logger.recordOutput("Elevator/Is at Target Position", isAtTargetPosition());
    }
}
