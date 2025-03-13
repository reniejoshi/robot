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

package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {
    // Motion Magic Constraints

    private static final double MAX_VELOCITY = 48; // Rotations per second
    private static final double MAX_ACCELERATION = MAX_VELOCITY * 4;
    private static final double MAX_JERK = MAX_ACCELERATION * 4;

    // States

    public enum IndexerState {
        DISABLED(0),
        COLLECTING(MAX_VELOCITY),
        PASSING(MAX_VELOCITY),
        EJECTING(-MAX_VELOCITY);

        public final double velocity;

        IndexerState(double velocity) {
            this.velocity = velocity;
        }
    }

    // Configuration

    public static final TalonFXConfiguration configuration = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(MAX_VELOCITY)
                .withMotionMagicAcceleration(MAX_ACCELERATION)
                .withMotionMagicJerk(MAX_JERK)
        ).withSlot0(
            new Slot0Configs()
                .withKP(0.083374)
                .withKS(0.17818)
                .withKV(0.12464)
                .withKA(0.0039997)
        ).withAudio(
            new AudioConfigs()
                .withBeepOnBoot(true)
                .withBeepOnConfig(true)
        );

}
