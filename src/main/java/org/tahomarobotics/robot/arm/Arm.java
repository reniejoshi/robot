package org.tahomarobotics.robot.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class Arm {
    final ArmSubsystem arm;

    public Arm() {
        this(new ArmSubsystem());
    }

    Arm(ArmSubsystem arm) {
        this.arm = arm;
    }

    // Command factory methods

    public Command setArmPosition(DoubleSupplier rightYSupplier) {
        return arm.run(() -> arm.setArmPosition(rightYSupplier));
    }

    public Command setWristPositionClockwise() {
        return arm.run(arm::setWristPositionClockwise);
    }

    public Command setWristPositionCounterclockwise() {
        return arm.run(arm::setWristPositionCounterclockwise);
    }

    public void setDefaultCommand(Command command) {
        arm.setDefaultCommand(command);
    }
}