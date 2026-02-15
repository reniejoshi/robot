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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;

public class DiffyArmSubsystem extends AbstractSubsystem {
    // Motors
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    // Absolute encoders
    private final CANcoder elbowEncoder;
    private final CANcoder wristEncoder;

    // Control requests
    private final PositionVoltage positionControl = new PositionVoltage(0);

    // Status signals
    private StatusSignal<Angle> elbowPosition;
    private StatusSignal<Angle> wristPosition;

    // Target angles
    private double targetElbowPosition = 0;
    private double targetWristPosition = 0;

    public DiffyArmSubsystem() {
        // Initialize hardware
        topMotor = new TalonFX(RobotMap.DIFFY_ARM_TOP_MOTOR);
        bottomMotor = new TalonFX(RobotMap.DIFFY_ARM_BOTTOM_MOTOR);
        elbowEncoder = new CANcoder(RobotMap.DIFFY_ARM_TOP_ENCODER);
        wristEncoder = new CANcoder(RobotMap.DIFFY_ARM_BOTTOM_ENCODER);

        elbowPosition = elbowEncoder.getPosition();
        wristPosition = wristEncoder.getPosition();

        org.tinylog.Logger.info("Creating an instance of DiffyArmSubsystem...");
    }

    DiffyArmSubsystem(TalonFX topMotor, TalonFX bottomMotor, CANcoder elbowEncoder, CANcoder wristEncoder) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
        this.elbowEncoder = elbowEncoder;
        this.wristEncoder = wristEncoder;
    }

    // Setters

    public void setArmPosition(DoubleSupplier leftXSupplier, DoubleSupplier leftYSupplier) {
        double x = leftXSupplier.getAsDouble();
        Logger.recordOutput("Diffy Arm/Left X Axis", x);
        double y = leftYSupplier.getAsDouble();
        Logger.recordOutput("Diffy Arm/Left Y Axis", y);

        double elbowIncrease = x * DiffyArmConstants.ELBOW_INCREMENT.in(Degrees);
        double wristIncrease = y * DiffyArmConstants.WRIST_INCREMENT.in(Degrees);

        targetElbowPosition = MathUtil.clamp(
            targetElbowPosition + elbowIncrease,
             DiffyArmConstants.ELBOW_MIN_POSITION.in(Degrees),
             DiffyArmConstants.ELBOW_MAX_POSITION.in(Degrees)
        );
        targetWristPosition = MathUtil.clamp(
            targetWristPosition + wristIncrease,
            DiffyArmConstants.WRIST_MIN_POSITION.in(Degrees),
            DiffyArmConstants.WRIST_MAX_POSITION.in(Degrees)
        );

        double topMotorPosition = (targetElbowPosition + targetWristPosition) * DiffyArmConstants.ELBOW_GEARBOX_RATIO;
        Logger.recordOutput("Diffy Arm/Top Motor Position", Degrees.of(topMotorPosition));
        double bottomMotorPosition = Math.abs((targetElbowPosition - targetWristPosition) * DiffyArmConstants.ELBOW_GEARBOX_RATIO);
        Logger.recordOutput("Diffy Arm/Bottom Motor Position", Degrees.of(bottomMotorPosition));

        topMotor.setControl(positionControl.withPosition(Degrees.of(topMotorPosition)));
        bottomMotor.setControl(positionControl.withPosition(Degrees.of(bottomMotorPosition)));
    }

    @Override
    public void subsystemPeriodic() {
        BaseStatusSignal.refreshAll(elbowEncoder.getPosition(), wristEncoder.getPosition());

        Logger.recordOutput("Diffy Arm/Target Elbow Position", targetElbowPosition);
        Logger.recordOutput("Diffy Arm/Target Wrist Position", targetWristPosition);
        Logger.recordOutput("Diffy Arm/Elbow Position", elbowPosition.getValue().in(Degrees));
        Logger.recordOutput("Diffy Arm/Wrist Position", wristPosition.getValue().in(Degrees));
    }
}
