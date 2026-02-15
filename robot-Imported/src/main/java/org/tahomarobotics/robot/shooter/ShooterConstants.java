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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import static edu.wpi.first.units.Units.*;

public class ShooterConstants {
    // Pivot limits
    public static final Angle PIVOT_MIN_POSITION = Degrees.of(45);
    public static final Angle PIVOT_MAX_POSITION = Degrees.of(135);

    public static final Voltage PIVOT_ZEROING_VOLTAGE = Volts.of(-2);
    public static final double PIVOT_ZEROING_WAIT_SECONDS = 0.5;
    public static final AngularVelocity PIVOT_ZERO_VELOCITY = RotationsPerSecond.of(0);
    public static final AngularVelocity PIVOT_ZERO_VELOCITY_THRESHOLD = RotationsPerSecond.of(0.1);
    public static final Angle PIVOT_ZERO_ANGLE = Degrees.of(0);

    public static final AngularVelocity FLYWHEEL_VELOCITY = RotationsPerSecond.of(300);

    // Placeholder TalonFXConfiguration object
    public static final TalonFXConfiguration createShooterMotorConfig = new TalonFXConfiguration();
}
