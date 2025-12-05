package org.tahomarobotics.robot.arm;

public class Arm {
    final ArmSubsystem arm;

    public Arm() {
        this(new ArmSubsystem());
    }

    Arm(ArmSubsystem arm) {
        this.arm = arm;
    }
}