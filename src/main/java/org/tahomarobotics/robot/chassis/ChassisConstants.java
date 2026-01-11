/*
 * MIT License
 *
 * Copyright (c) 2025 Bear Metal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import java.util.Map;

import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.robot.RobotMap.*;

public class ChassisConstants {
    public static final double CONTROLLER_ROTATIONAL_SENSITIVITY = 2d;
    public static final double CONTROLLER_TRANSLATIONAL_SENSITIVITY = 1.3d;
    public static final double CONTROLLER_DEADBAND = 0.09d;

    public static final double DRIVE_GEAR_RATIO = (54d / 14d) * (25d / 32d) * (30d / 15d);
    public static final double STEER_GEAR_RATIO = (49d / 7d) * (41d / 11d);
    public static final double COUPLING_GEAR_RATIO = 54d / 14d;

    private static final Distance WHEEL_RADIUS = Inches.of(2d);
    private static final double WHEEL_COF = 1.2;

    private static final SwerveModuleConstants.ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = SwerveModuleConstants.ClosedLoopOutputType.Voltage;
    private static final SwerveModuleConstants.ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = SwerveModuleConstants.ClosedLoopOutputType.Voltage;

    private static final Distance TRACK_WIDTH = Inches.of(20.75);
    private static final Distance WHEELBASE = Inches.of(20.75);

    private static final Distance HALF_TRACK_WIDTH = TRACK_WIDTH.div(2d);
    private static final Distance HALF_WHEELBASE = WHEELBASE.div(2d);

    private static final Distance ROBOT_RADIUS = Meters
        .of(Math.hypot(HALF_TRACK_WIDTH.in(Meters), HALF_WHEELBASE.in(Meters)));

    private static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1);

    public static final DCMotor STEER_MOTOR = new DCMotor(
        12, 4.05, 275, 1.4,
        Units.rotationsPerMinuteToRadiansPerSecond(7530), 1);

    public static final LinearVelocity MAX_LINEAR_VELOCITY = MetersPerSecond
        .of(DRIVE_MOTOR.withReduction(DRIVE_GEAR_RATIO).getSpeed(0, 12.0) * (WHEEL_RADIUS.in(Meters)));
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond
        .of(MAX_LINEAR_VELOCITY.in(MetersPerSecond) / ROBOT_RADIUS.in(Meters));

    private static final SwerveModuleConstants.DriveMotorArrangement DRIVE_MOTOR_TYPE = SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated;
    private static final SwerveModuleConstants.SteerMotorArrangement STEER_MOTOR_TYPE = SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated;
    private static final SwerveModuleConstants.SteerFeedbackType STEER_FEEDBACK_TYPE = SwerveModuleConstants.SteerFeedbackType.FusedCANcoder;

    private static final TalonFXConfiguration DRIVE_MOTOR_CONFIG = new TalonFXConfiguration();
    private static final TalonFXConfiguration STEER_MOTOR_CONFIG = new TalonFXConfiguration();
    private static final CANcoderConfiguration CANCODER_CONFIG = new CANcoderConfiguration();

    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
        .withCANBusName(CANBUS_NAME)
        .withPigeon2Id(PIGEON);

    private static final Current DRIVE_SLIP_CURRENT = Amps.of(120.0);

    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
        .withKP(100)
        .withKI(0)
        .withKD(0.5)
        .withKS(0.1)
        .withKV(1.59)
        .withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
        .withKP(10)
        .withKI(0)
        .withKD(0)
        .withKS(0)
        .withKV(0.124);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MODULE_CONSTANTS_FACTORY = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
        .withSteerMotorGearRatio(STEER_GEAR_RATIO)
        .withCouplingGearRatio(COUPLING_GEAR_RATIO)
        .withWheelRadius(WHEEL_RADIUS)
        .withSteerMotorGains(STEER_GAINS)
        .withDriveMotorGains(DRIVE_GAINS)
        .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
        .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
        .withSlipCurrent(DRIVE_SLIP_CURRENT)
        .withSpeedAt12Volts(MAX_LINEAR_VELOCITY)
        .withSteerMotorType(STEER_MOTOR_TYPE)
        .withDriveMotorType(DRIVE_MOTOR_TYPE)
        .withFeedbackSource(STEER_FEEDBACK_TYPE)
        .withDriveMotorInitialConfigs(DRIVE_MOTOR_CONFIG)
        .withSteerMotorInitialConfigs(STEER_MOTOR_CONFIG)
        .withEncoderInitialConfigs(CANCODER_CONFIG);

    private static final Map<ModuleId, Distance> DISTANCE_X = Map.of(
        FRONT_LEFT_MODULE, HALF_WHEELBASE,
        FRONT_RIGHT_MODULE, HALF_WHEELBASE,
        BACK_LEFT_MODULE, HALF_WHEELBASE.unaryMinus(),
        BACK_RIGHT_MODULE, HALF_WHEELBASE.unaryMinus());

    private static final Map<ModuleId, Distance> DISTANCE_Y = Map.of(
        FRONT_LEFT_MODULE, HALF_TRACK_WIDTH,
        FRONT_RIGHT_MODULE, HALF_TRACK_WIDTH.unaryMinus(),
        BACK_LEFT_MODULE, HALF_TRACK_WIDTH,
        BACK_RIGHT_MODULE, HALF_TRACK_WIDTH.unaryMinus());

    public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getModuleConfig(
        ModuleId moduleId, Angle steerOffset) {

        var swerveModuleConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(moduleId.steerId(),
                                                                                moduleId.driveId(),
                                                                                moduleId.cancoderId(),
                                                                                steerOffset,
                                                                                DISTANCE_X.get(moduleId),
                                                                                DISTANCE_Y.get(moduleId),
                                                                                false,
                                                                                false,
                                                                                false);

        return swerveModuleConfig;
    }

    // Robot physical constants
    private static final Mass BATTERY_MASS = Pounds.of(13.6);
    private static final Mass BUMPER_MASS = Pounds.of(16.0);
    private static final Mass CHASSIS_MASS = Pounds.of(50.0);
    private static final Mass ROBOT_MASS = CHASSIS_MASS.plus(BATTERY_MASS).plus(BUMPER_MASS);

    private static final Distance BUMPER_SIZE = Inches.of(34.086);
    private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.05);
    private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.25);
    private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.25);
}