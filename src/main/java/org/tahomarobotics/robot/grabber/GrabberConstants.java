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

package org.tahomarobotics.robot.grabber;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.tahomarobotics.robot.util.identity.Identity;

public class GrabberConstants {
    public static final double CORAL_COLLECT_VELOCITY = -20;
    public static final double ALGAE_COLLECT_VELOCITY = -10;
    public static final double SCORING_VELOCITY = 50;
    public static final double CORAL_HOLD_VOLTAGE = 0;
    public static final double ALGAE_HOLD_VOLTAGE = -1.35;

    public static final double CORAL_COLLECTION_DELAY = 0.25;
    public static final double ALGAE_COLLECTION_DELAY = 0.1;

    public static final double GEAR_REDUCTION;

    static {
        switch (Identity.robotID) {
            case BEEF, BEARRACUDA -> GEAR_REDUCTION = (8d / 26d);
            default -> GEAR_REDUCTION = (10d / 26d);
        }
    }

    public static double CORAL_COLLECTION_CURRENT_THRESHOLD;
    public static double ALGAE_COLLECTION_CURRENT_THRESHOLD;

    // TODO: Tune with Bearracuda
    static {
        CORAL_COLLECTION_CURRENT_THRESHOLD = switch (Identity.robotID) {
            case BEEF -> 15;
            case BEARRACUDA -> 16;
            default -> 20;
        };
        ALGAE_COLLECTION_CURRENT_THRESHOLD = switch (Identity.robotID) {
            case BEEF -> 60;
            case BEARRACUDA -> 60;
            default -> 60;
        };
    }

    // -- States --

    public enum GrabberState {
        DISABLED(MotionType.NONE, 0),
        CORAL_HOLDING(MotionType.VOLTAGE, CORAL_HOLD_VOLTAGE),
        CORAL_COLLECTING(MotionType.VELOCITY, CORAL_COLLECT_VELOCITY),
        ALGAE_HOLDING(MotionType.VOLTAGE, ALGAE_HOLD_VOLTAGE),
        ALGAE_COLLECTING(MotionType.VELOCITY, ALGAE_COLLECT_VELOCITY),
        SCORING(MotionType.VELOCITY, SCORING_VELOCITY),
        L1_SCORING(MotionType.VELOCITY, -SCORING_VELOCITY);

        public final MotionType type;
        public final double value;

        GrabberState(MotionType type, double value) {
            this.type = type;
            this.value = value;
        }

        public enum MotionType {
            VELOCITY, VOLTAGE, NONE
        }
    }

    // -- Configuration --

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
        // SysId'd 02/11
        .withSlot0(new Slot0Configs()
                       .withKP(1.048663)
                       .withKS(0.05988)
                       .withKV(0.37575)
                       .withKA(0.0053009))
        .withMotorOutput(new MotorOutputConfigs()
                             .withNeutralMode(NeutralModeValue.Brake)
                             .withInverted(InvertedValue.CounterClockwise_Positive))
        .withMotionMagic(new MotionMagicConfigs()
                             .withMotionMagicCruiseVelocity(5)
                             .withMotionMagicAcceleration(100)
                             .withMotionMagicJerk(5000))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / GEAR_REDUCTION))
        .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
