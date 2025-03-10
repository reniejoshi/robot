package org.tahomarobotics.robot.windmill;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.identity.Identity;

public class WindmillConstants {
    // -- Elevator --

    // States

    public enum TrajectoryState {
        HIGH_DESCORE( Units.inchesToMeters(15.257), Units.degreesToRadians( 127.820)),  // ALGAE: B button
        LOW_DESCORE(  Units.inchesToMeters( 0.815), Units.degreesToRadians( 127.742)),  // ALGAE: A button
        COLLECT(      Units.inchesToMeters(17.069), Units.degreesToRadians( 267.809)),  // X button toggle
        STOW(         Units.inchesToMeters( 1.901), Units.degreesToRadians(  89.913)),  // X button toggle
        L4(           Units.inchesToMeters(40.344), Units.degreesToRadians( 120.548)),  // Y button
        L3(           Units.inchesToMeters(17.500), Units.degreesToRadians( 124.398)),  // CORAL: B button
        L2(           Units.inchesToMeters( 1.364), Units.degreesToRadians( 124.791)),  // CORAL: A button
        START(        Units.inchesToMeters( 0.468), Units.degreesToRadians(  90.089));  // startup only

        public final double elev;
        public final double arm;
        public final WindmillState state;

        TrajectoryState(double elev, double arm) {
            this.elev = elev;
            this.arm = arm;
            this.state = new WindmillState(0,
                                           new WindmillState.ElevatorState(elev,0,0),
                                           new WindmillState.ArmState(arm, 0,0));
        }
    }

    public static final double SMALL_PULLBACK = Units.degreesToRadians(10.0);
    public static final double STANDARD_PULLBACK = Units.degreesToRadians(20.0);
    public static final double LARGE_PULLBACK = Units.degreesToRadians(30.0);

    // Gearing
    public static final double ELEVATOR_GEAR_REDUCTION;

    // Pulley
    public static final double ELEVATOR_MAIN_PULLEY_RADIUS = Units.inchesToMeters(1.1056);
    public static final double ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE = 2 * Math.PI * ELEVATOR_MAIN_PULLEY_RADIUS;

    // Poses
    public static final double ELEVATOR_MAX_POSE = 1.035; // Meters
    public static final double ELEVATOR_MIN_POSE = 0.01; // Meters

    public static final double ELEVATOR_COLLECT_POSE = 0.43;
    public static final double ELEVATOR_HIGH_POSE = 1.03; // Meters
    public static final double ELEVATOR_MID_POSE = 0.45; // Meters
    public static final double ELEVATOR_LOW_POSE = 0.05; // Meters

    // Tolerances

    public static final double ELEVATOR_POSITION_TOLERANCE = 0.005; // Meters
    public static final double ELEVATOR_VELOCITY_TOLERANCE = 0.01; // Meters / sec

    // Motion

    public static final double ELEVATOR_MAX_VELOCITY = 4; // Meters / sec
    public static final double ELEVATOR_MAX_ACCELERATION = ELEVATOR_MAX_VELOCITY * 4; // Meters / sec^2
    public static final double ELEVATOR_MAX_JERK = ELEVATOR_MAX_ACCELERATION * 4.0; // Meters / sec^3

    // -- Arm --

    // Gearing
    public static final double ARM_BELT_REDUCTION = 18d / 72d;
    public static final double ARM_GEAR_REDUCTION;

    // Poses

    public static final double ARM_CALIBRATION_POSE = 0.25; // Rotation (used for motor only)
    public static final double ARM_COLLECT_POSE = Units.rotationsToRadians(0.75);
    public static final double ARM_UPRIGHT_POSE = Units.rotationsToRadians(0.25);
    public static final double ARM_TEST_POSE = Units.rotationsToRadians(0); // TODO: temporary

    // Tolerances

    public static final double ARM_POSITION_TOLERANCE = Units.rotationsToRadians(0.005); // Radians
    public static final double ARM_VELOCITY_TOLERANCE = Units.rotationsToRadians(0.01); // Radians / sec

    // Motion

    public static final double ARM_MAX_VELOCITY = Units.rotationsToRadians(1.5); // Radians / sec
    public static final double ARM_MAX_ACCELERATION = Units.rotationsToRadians(4); // Radians / sec^2
    public static final double ARM_MAX_JERK = ARM_MAX_ACCELERATION * 4; // Radians / sec^3

    // Constants

    public static final double ARM_LENGTH = 0.6540246; // Meters

    public static final double ELEVATOR_ZEROING_VOLTAGE = -1;
    public static final double ELEVATOR_ZEROING_TIMEOUT = 0.2;

    // -- Constraints --

    public static final double END_EFFECTOR_MIN_HEIGHT = -0.25; // Bottom-most point the carriage hits
    public static final double END_EFFECTOR_MAX_HEIGHT = ELEVATOR_MAX_POSE + ARM_LENGTH;

    // -- Identity --

    static {
        switch (Identity.robotID) {
            case BEEF, BEARRACUDA -> {
                ELEVATOR_GEAR_REDUCTION = 10d / 52d;
                ARM_GEAR_REDUCTION = 8d / 60d * 24d / 50d * ARM_BELT_REDUCTION;
            }
            default -> {
                ELEVATOR_GEAR_REDUCTION = 12d / 52d;
                ARM_GEAR_REDUCTION = 10d / 60d * 24d / 50d * ARM_BELT_REDUCTION;
            }
        }
    }

    // -- Configurations --

    static final TalonFXConfiguration elevatorMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                // SysId'd 1/17/2025
                .withKP(50.438)
                .withKI(100)
                .withKS(0.097499)
                .withKV(3.3441)
                .withKA(0.16116)
                .withKG(0.084958)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(false)
                .withSupplyCurrentLimitEnable(false)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(ELEVATOR_MAX_VELOCITY)
                .withMotionMagicAcceleration(ELEVATOR_MAX_ACCELERATION)
                .withMotionMagicJerk(ELEVATOR_MAX_JERK)
        ).withClosedLoopGeneral(
            new ClosedLoopGeneralConfigs()
                .withContinuousWrap(false)
        ).withFeedback(new FeedbackConfigs()
                           .withSensorToMechanismRatio(1 / ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE)
                           .withFeedbackRemoteSensorID(RobotMap.ELEVATOR_ENCODER)
                           .withFeedbackSensorSource(
                               RobotConfiguration.RIO_PHOENIX_PRO ? FeedbackSensorSourceValue.FusedCANcoder : FeedbackSensorSourceValue.RemoteCANcoder)
        ).withAudio(
            new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true)
        );

    static final TalonFXConfiguration armMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                // Hand-tuned 02/06/25
                .withKP(80)
                .withKD(0)
                .withKS(0)
                .withKV(0)
                .withKA(0)
                .withKG(0.4)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(ARM_MAX_VELOCITY)
                .withMotionMagicAcceleration(ARM_MAX_ACCELERATION)
                .withMotionMagicJerk(ARM_MAX_JERK)
        ).withClosedLoopGeneral(
            new ClosedLoopGeneralConfigs()
                .withContinuousWrap(true)
        ).withFeedback(new FeedbackConfigs()
                           .withSensorToMechanismRatio(1 / ARM_BELT_REDUCTION)
                           .withFeedbackRemoteSensorID(RobotMap.ARM_ENCODER)
                           .withFeedbackSensorSource(
                               RobotConfiguration.RIO_PHOENIX_PRO ? FeedbackSensorSourceValue.FusedCANcoder : FeedbackSensorSourceValue.RemoteCANcoder)
        ).withAudio(
            new AudioConfigs()
                .withBeepOnBoot(true)
                .withBeepOnConfig(true)
        );

    static final CANcoderConfiguration armEncoderConfiguration = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
                              .withAbsoluteSensorDiscontinuityPoint(1)
                              .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        );
}
