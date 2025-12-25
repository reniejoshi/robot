package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;

public class ArmSubsystem extends AbstractSubsystem {
    // Motors
    private final TalonFX armMotor = new TalonFX(RobotMap.ARM_MOTOR);
    private final TalonFX wristMotor = new TalonFX(RobotMap.WRIST_MOTOR);

    // Control requests
    private final PositionVoltage positionControl = new PositionVoltage(0);

    // Status signals
    private final StatusSignal<Angle> armMotorPosition = armMotor.getPosition();
    private final StatusSignal<Angle> wristMotorPosition = wristMotor.getPosition();

    // Target position variables
    private double targetArmPosition = 0;
    private double targetWristPosition = 0;

    // Setters

    public void setArmPosition(DoubleSupplier rightYSupplier) {
        double y = rightYSupplier.getAsDouble();
        Logger.recordOutput("Arm/Right Y Axis", y);

        double increase = y * ArmConstants.ARM_INCREMENT.in(Degrees);
        targetArmPosition = MathUtil.clamp(
            targetArmPosition + increase,
            ArmConstants.ARM_MIN_POSITION.in(Degrees),
            ArmConstants.ARM_MAX_POSITION.in(Degrees)
        );
        armMotor.setControl(positionControl.withPosition(Degrees.of(targetArmPosition)));
    }

    public void setWristPositionClockwise() {
        moveWristToTargetPosition(ArmConstants.WRIST_INCREMENT.in(Degrees));
    }

    public void setWristPositionCounterclockwise() {
        moveWristToTargetPosition(-ArmConstants.WRIST_INCREMENT.in(Degrees));
    }

    // Helpers

    private void moveWristToTargetPosition(double increase) {
        targetWristPosition = MathUtil.clamp(
            targetWristPosition + increase,
            ArmConstants.WRIST_MIN_POSITION.in(Degrees),
            ArmConstants.WRIST_MAX_POSITION.in(Degrees)
        );
        wristMotor.setControl(positionControl.withPosition(Degrees.of(targetWristPosition)));
    }

    @Override
    public void subsystemPeriodic() {
        StatusSignal.refreshAll(armMotorPosition, wristMotorPosition);

        Logger.recordOutput("Arm/Arm Motor Position", armMotorPosition.getValue().in(Degrees));
        Logger.recordOutput("Arm/Target Arm Position", targetArmPosition);
        Logger.recordOutput("Arm/Wrist Motor Position", wristMotorPosition.getValue().in(Degrees));
        Logger.recordOutput("Arm/Target Wrist Position", targetWristPosition);
    }
}