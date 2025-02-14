package org.tahomarobotics.robot.grabber;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class GrabberConstants {
    public static final double CORAL_COLLECT_VELOCITY = -15;
    public static final double CORAL_EJECT_VELOCITY = 15;
    public static final double CORAL_HOLD_VOLTAGE = -0.0;
    public static final double CORAL_GRABBED_VELOCITY_THRESHOLD = 0.2;
    public static final double CORAL_GRABBED_CHECK_TIME = 0.1;
    public static final double CORAL_EJECTED_CHECK_TIME = 0.1;
    public static final double CORAL_EJECTED_VELOCITY_THRESHOLD = 10;

    public static final double ALGAE_HOLD_VOLTAGE = 0.5;
    public static final double ALGAE_COLLECT_VELOCITY = -30;
    public static final double ALGAE_EJECT_VELOCITY = 30;
    public static final double ALGAE_COLLECT_VELOCITY_THRESHOLD = 0.2;
    public static final double ALGAE_EJECT_VELOCITY_THRESHOLD = 0.2;
    public static final double ALGAE_EJECT_CHECK_TIME = 0.2;
    public static final double ALGAE_COLLECT_CHECK_TIME = 0.2;

    public static final double GRABBER_GEAR_REDUCTION = 1.0;

    public static final TalonFXConfiguration grabberMotorConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
            .withKP(0.165)
            .withKV(0.11586)
            .withKA(0.039947))
        .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
        .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(5)
                    .withMotionMagicAcceleration(15)
                    .withMotionMagicJerk(50))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / GRABBER_GEAR_REDUCTION))
        .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}