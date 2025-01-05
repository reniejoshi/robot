package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.tahomarobotics.robot.RobotConfiguration;

public class ChassisConstants {

    public static final double TRACK_WIDTH = 0.5816;
    public static final double WHEELBASE = 0.8194;
    public static final double HALF_TRACK_WIDTH = TRACK_WIDTH / 2;
    public static final double HALF_WHEELBASE = WHEELBASE / 2;

    public static final double WHEEL_RADIUS = 0.0508;
    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
    public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double DRIVE_POSITION_COEFFICIENT = WHEEL_CIRCUMFERENCE * DRIVE_REDUCTION;

    public static final DCMotor SWERVE_MOTOR = DCMotor.getKrakenX60(1);
    public static final double MAX_VELOCITY = SWERVE_MOTOR.freeSpeedRadPerSec * DRIVE_REDUCTION * WHEEL_RADIUS * 0.5;
    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(HALF_TRACK_WIDTH, HALF_WHEELBASE);
    public static final double ACCELERATION_LIMIT = 3.0;

    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(HALF_WHEELBASE, HALF_TRACK_WIDTH);
    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(HALF_WHEELBASE, -HALF_TRACK_WIDTH);
    public static final Translation2d BACK_LEFT_OFFSET = new Translation2d(-HALF_WHEELBASE, HALF_TRACK_WIDTH);
    public static final Translation2d BACK_RIGHT_OFFSET = new Translation2d(-HALF_WHEELBASE, -HALF_TRACK_WIDTH);

    public static final double kV_DRIVE = (2 * Math.PI) / SWERVE_MOTOR.KvRadPerSecPerVolt;

    public static final TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(0.15)
                    .withKV(kV_DRIVE))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(120)
                    .withMotionMagicJerk(360)
            )
            .withAudio(new AudioConfigs()
                    .withBeepOnBoot(true)
                    .withBeepOnConfig(true));

    public static final TalonFXConfiguration steerMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(8.0)
                    .withKI(0.01)
                    .withKD(0.16)
            )
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(25)
                    .withMotionMagicJerk(100)
            )
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs() {{
                ContinuousWrap = true;
            }})
            .withFeedback(new FeedbackConfigs() {{
                              if (RobotConfiguration.CANIVORE_PHOENIX_PRO) {
                                  FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                                  RotorToSensorRatio = 1 / STEER_REDUCTION;
                              } else {
                                  FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                              }
                          }}
            )
            .withAudio(new AudioConfigs()
                    .withBeepOnBoot(true)
                    .withBeepOnConfig(true));

    public static final MagnetSensorConfigs encoderConfiguration = new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(1)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

    public static double clampAccel(double value) {
        return MathUtil.clamp(value, -ACCELERATION_LIMIT, ACCELERATION_LIMIT);
    }
}
