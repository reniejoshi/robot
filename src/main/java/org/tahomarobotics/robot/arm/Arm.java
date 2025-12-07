package org.tahomarobotics.robot.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public class Arm {
    final ArmSubsystem arm;

    public Arm() {
        this(new ArmSubsystem());
    }

    Arm(ArmSubsystem arm) {
        this.arm = arm;
    }

    // Command factory methods

    public Command setArmPosition(Angle position) {
        return arm.runOnce(() -> arm.setArmPosition(position));
    }

    public Command setWristPosition(Angle position) {
        return arm.runOnce(() -> arm.setWristPosition(position));
    }
}