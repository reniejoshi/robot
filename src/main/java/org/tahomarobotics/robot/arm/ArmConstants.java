package org.tahomarobotics.robot.arm;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class ArmConstants {
    // Arm limits
    public static final Angle ARM_MIN_POSITION = Degrees.of(0);
    public static final Angle ARM_MAX_POSITION = Degrees.of(180);

    // Wrist limits
    public static final Angle WRIST_MIN_POSITION = Degrees.of(0);
    public static final Angle WRIST_MAX_POSITION = Degrees.of(300);

    // Increment values
    public static final double ARM_INCREMENT = 1;
    public static final double WRIST_INCREMENT = 1;
}