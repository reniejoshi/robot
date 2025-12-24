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

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
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
    private final CANcoder topEncoder;
    private final CANcoder bottomEncoder;

    // Control requests
    private final PositionVoltage positionControl = new PositionVoltage(0);

    public DiffyArmSubsystem() {
        // Initialize hardware
        topMotor = new TalonFX(RobotMap.DIFFY_ARM_TOP_MOTOR);
        bottomMotor = new TalonFX(RobotMap.DIFFY_ARM_BOTTOM_MOTOR);
        topEncoder = new CANcoder(RobotMap.DIFFY_ARM_TOP_ENCODER);
        bottomEncoder = new CANcoder(RobotMap.DIFFY_ARM_BOTTOM_ENCODER);

        org.tinylog.Logger.info("Creating an instance of DiffyArmSubsystem...");
    }

    DiffyArmSubsystem(TalonFX topMotor, TalonFX bottomMotor, CANcoder topEncoder, CANcoder bottomEncoder) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
        this.topEncoder = topEncoder;
        this.bottomEncoder = bottomEncoder;
    }

    // Setters

    public void setArmPosition(DoubleSupplier rightXSupplier, DoubleSupplier rightYSupplier) {
        double x = rightXSupplier.getAsDouble();
        Logger.recordOutput("Diffy Arm/Right X Axis", x);
        double y = rightYSupplier.getAsDouble();
        Logger.recordOutput("Diffy Arm/Right Y Axis", y);

        double elbowAngle = x * DiffyArmConstants.ARM_MAX_POSITION.in(Units.Degrees);
        Logger.recordOutput("Diffy Arm/Elbow Angle", Degrees.of(elbowAngle));
        double wristAngle = y * DiffyArmConstants.WRIST_MAX_POSITION.in(Degrees);
        Logger.recordOutput("Diffy Arm/Wrist Angle", Degrees.of(wristAngle));

        double topMotorPosition = (elbowAngle + wristAngle) * DiffyArmConstants.ELBOW_GEARBOX_RATIO;
        Logger.recordOutput("Diffy Arm/Top Motor Position", Degrees.of(topMotorPosition));
        double bottomMotorPosition = (elbowAngle - wristAngle) * DiffyArmConstants.ELBOW_GEARBOX_RATIO;
        Logger.recordOutput("Diffy Arm/Bottom Motor Position", Degrees.of(bottomMotorPosition));

        topMotor.setControl(positionControl.withPosition(Degrees.of(topMotorPosition)));
        bottomMotor.setControl(positionControl.withPosition(Degrees.of(bottomMotorPosition)));
    }

    @Override
    public void subsystemPeriodic() {

    }
}
