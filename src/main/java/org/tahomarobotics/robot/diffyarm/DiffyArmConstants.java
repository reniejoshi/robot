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

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class DiffyArmConstants {
    // Arm limits
    public static final Angle ARM_MIN_POSITION = Degrees.of(0);
    public static final Angle ARM_MAX_POSITION = Degrees.of(180);

    // Wrist limits
    public static final Angle WRIST_MIN_POSITION = Degrees.of(0);
    public static final Angle WRIST_MAX_POSITION = Degrees.of(300);

    public static final double ELBOW_GEARBOX_RATIO = 60d / 10d * 48d / 12d;
    public static final double WRIST_GEARBOX_RATIO = ELBOW_GEARBOX_RATIO * 45d / 15d;

    public static final double VOLTAGE = 12.0;
}
