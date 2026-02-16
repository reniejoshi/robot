package org.tahomarobotics.robot.collector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class CollectorConstants {
    // -- Zeroing constants --
    public static final Voltage PIVOT_ZEROING_VOLTAGE = Volts.of(-2);
    public static final double PIVOT_ZEROING_WAIT_SECONDS = 0.1;
    public static final AngularVelocity PIVOT_ZERO_VELOCITY = RotationsPerSecond.of(0);
    public static final AngularVelocity PIVOT_ZER0_VELOCITY_THRESHOLD = RotationsPerSecond.of(0.01);
    public static final Angle PIVOT_ZERO_ANGLE = Degrees.of(0);

    // -- Roller motor constants --
    public static final AngularVelocity COLLECTING_VELOCITY = RotationsPerSecond.of(10);
    public static final AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(0);
    public static final AngularVelocity EJECTING_VELOCITY = RotationsPerSecond.of(-10);

    // -- Pivot motor constants --
    public static final Angle DEPLOYED_ANGLE = Degrees.of(0);
    public static final Angle STOWED_ANGLE = Degrees.of(90);

    public static final TalonFXConfiguration createCollectorMotorConfig = new TalonFXConfiguration();    
}
