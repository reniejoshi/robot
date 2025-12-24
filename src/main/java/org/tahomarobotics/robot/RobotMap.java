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

package org.tahomarobotics.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class RobotMap {
    public final static int PIGEON = 0; // Internal IMU. Will not be used in Extra Programming Projects (Robot)

    public final static int ARM_MOTOR = 1;
    public final static int WRIST_MOTOR = 2;

    public final static int ELEVATOR_LEFT_MOTOR = 3;
    public final static int ELEVATOR_RIGHT_MOTOR = 4;

    public final static int DIFFY_ARM_TOP_MOTOR = 5;
    public final static int DIFFY_ARM_BOTTOM_MOTOR = 6;
    public final static int DIFFY_ARM_TOP_ENCODER = 7;
    public final static int DIFFY_ARM_BOTTOM_ENCODER = 8;
}
