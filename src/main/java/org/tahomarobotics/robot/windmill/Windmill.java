package org.tahomarobotics.robot.windmill;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.persistent.CalibrationData;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tahomarobotics.robot.windmill.commands.WindmillCommands;
import org.tahomarobotics.robot.windmill.commands.WindmillMoveCommand;
import org.tinylog.Logger;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.robot.windmill.WindmillConstants.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Windmill extends SubsystemIF {
    private static final Windmill INSTANCE = new Windmill();

    // -- Member Variables --

    // Hardware

    private final TalonFX elevatorLeftMotor;
    private final TalonFX elevatorRightMotor;
    private final TalonFX armMotor;
    private final CANcoder elevatorEncoder;
    private final CANcoder armEncoder;

    // Status Signals

    private final StatusSignal<Angle> elevatorPosition, armPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity, armVelocity;
    private final StatusSignal<Current> elevatorCurrent, armCurrent;

    // Control Requests

    private final MotionMagicVoltage elevatorPositionControl = new MotionMagicVoltage(0);
    private final MotionMagicVoltage armPositionControl = new MotionMagicVoltage(0);

    // State

    private double targetHeight;
    private double targetAngle;

    private boolean elevatorCalibrated = true, armCalibrated = true;
    private double armOffset;
    private CalibrationData<Boolean> elevatorCalibration = null;
    private CalibrationData<Boolean> armCalibration = null;

    // Trajectory

    private final StringSubscriber sub;
    private final Field2d field = new Field2d();
    private WindmillTrajectory networkTablesTrajectory;

    // -- Initialization --

    private Windmill() {
        // Create Hardware

        elevatorLeftMotor = new TalonFX(RobotMap.ELEVATOR_LEFT_MOTOR);
        elevatorRightMotor = new TalonFX(RobotMap.ELEVATOR_RIGHT_MOTOR);
        elevatorEncoder = new CANcoder(RobotMap.ELEVATOR_ENCODER);

        armMotor = new TalonFX(RobotMap.ARM_MOTOR);
        armEncoder = new CANcoder(RobotMap.ARM_ENCODER);

        // Configure hardware

        RobustConfigurator.tryConfigureCANcoder("Arm Encoder", armEncoder, armEncoderConfiguration);
        RobustConfigurator.tryConfigureTalonFX("Elevator Left Motor", elevatorLeftMotor, elevatorMotorConfiguration);
        elevatorRightMotor.setControl(new Follower(RobotMap.ELEVATOR_LEFT_MOTOR, true));
        RobustConfigurator.tryConfigureTalonFX("Arm Motor", armMotor, armMotorConfiguration);

        // Load previous calibrations

        if (RobotBase.isReal()) {
            elevatorCalibration = new CalibrationData<>("ElevatorCalibration", false);
            armCalibration = new CalibrationData<>("ArmCalibration", false);

            elevatorCalibrated = elevatorCalibration.get();
            armCalibrated = armCalibration.isCalibrated();

            applyArmOffset();

            if (!elevatorCalibrated) {
                Logger.error("Elevator is not calibrated, cannot use until calibration is completed!");
            }
            if (!armCalibrated) {
                Logger.error("Arm is not calibrated, cannot use until calibration is completed!");
            }
        }

        // Status Signals

        elevatorPosition = elevatorLeftMotor.getPosition();
        elevatorVelocity = elevatorLeftMotor.getVelocity();
        elevatorCurrent = elevatorLeftMotor.getSupplyCurrent();

        armPosition = armMotor.getPosition();
        armVelocity = armMotor.getVelocity();
        armCurrent = armMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
            elevatorPosition, elevatorVelocity, elevatorCurrent,
            armPosition, armVelocity, armCurrent,
            elevatorEncoder.getPosition(), elevatorEncoder.getVelocity(),
            armEncoder.getPosition(), armEncoder.getVelocity(),
            elevatorLeftMotor.getMotorVoltage(), armMotor.getMotorVoltage()
        );

        ParentDevice.optimizeBusUtilizationForAll(
            elevatorLeftMotor, elevatorRightMotor, elevatorEncoder, armMotor, armEncoder);

        // Sub
        sub = NetworkTableInstance.getDefault().getStringTopic("/SmartDashboard/BEEF/Hash")
                                  .subscribe("");
    }

    public static Windmill getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        // Publish calibration commands
        SmartDashboard.putData("Calibrate Elevator", WindmillCommands.createCalibrateElevatorCommand(this));
        SmartDashboard.putData("Calibrate Arm", WindmillCommands.createCalibrateArmCommand(this));


        if (RobotConfiguration.IS_USING_TRAJECTORY_EDITOR) {
            // Trajectory Editor Field
            SmartDashboard.putData("Trajectory Editor Visualization", field);

            NetworkTableInstance.getDefault().addListener(
                sub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> {
                    networkTablesTrajectory = WindmillTrajectory.generateFromNetworkTables();
                    field.getObject("Trajectory").setTrajectory(networkTablesTrajectory.getTrajectory());
                }
            );
        }

        return this;
    }

    // -- Calibration --

    public void initializeElevatorCalibration() {
        RobustConfigurator.trySetMotorNeutralMode("Elevator Left Motor", elevatorLeftMotor, NeutralModeValue.Coast);
    }

    public void initializeArmCalibration() {
        RobustConfigurator.trySetMotorNeutralMode("Arm Left Motor", armMotor, NeutralModeValue.Coast);
    }

    public void finalizeElevatorCalibration() {
        elevatorEncoder.setPosition(0);

        elevatorCalibration.set(true);
        elevatorCalibrated = true;

        RobustConfigurator.trySetMotorNeutralMode("Elevator Motor", elevatorLeftMotor, NeutralModeValue.Brake);
    }

    public void finalizeArmCalibration() {
        // Divide by sensor coefficient to set accurate position
        armEncoder.setPosition(ARM_UPRIGHT_POSE / ARM_BELT_REDUCTION);

        armCalibration.set(true);
        armCalibrated = true;

        RobustConfigurator.trySetMotorNeutralMode("Arm Motor", armMotor, NeutralModeValue.Brake);
    }

    public void applyElevatorOffset() {
        RobustConfigurator.trySetMotorNeutralMode("Elevator Left Motor", elevatorLeftMotor, NeutralModeValue.Brake);
    }

    public void applyArmOffset() {
        RobustConfigurator.trySetMotorNeutralMode("Arm Motor", armMotor, NeutralModeValue.Brake);
    }

    // -- Getters --

    @Logged(name = "windmillPosition")
    public Translation2d getWindmillPosition() {
        double armAngleRotations = MathUtil.inputModulus(getArmPosition(), 0, 1);

        return WindmillKinematics.forwardKinematics(
            elevatorPosition.getValueAsDouble(),
            Units.rotationsToRadians(armAngleRotations),
            true
        );
    }

    @Logged(name = "windmillX")
    public double getWindmillPositionX() {
        return getWindmillPosition().getX();
    }

    @Logged(name = "windmillY")
    public double getWindmillPositionY() {
        return getWindmillPosition().getY();
    }

    public List<BaseStatusSignal> getStatusSignals() {
        return List.of(elevatorPosition, armPosition, elevatorVelocity, armVelocity, elevatorCurrent, armCurrent);
    }

    public WindmillState getCurrentState() {
        WindmillState.ElevatorState elevatorState = new WindmillState.ElevatorState(
            elevatorPosition.getValueAsDouble(),
            elevatorVelocity.getValueAsDouble(),
            elevatorLeftMotor.getAcceleration().refresh().getValueAsDouble()
        );

        WindmillState.ArmState armState = new WindmillState.ArmState(
            armPosition.getValueAsDouble(),
            armVelocity.getValueAsDouble(),
            armMotor.getAcceleration().refresh().getValueAsDouble()
        );

        return new WindmillState(0, elevatorState, armState);
    }

    // Elevator

    @Logged(name = "elevatorHeight")
    public double getElevatorHeight() {
        return elevatorPosition.getValueAsDouble();
    }

    @Logged(name = "elevatorTarget")
    public double getElevatorTarget() {
        return targetHeight;
    }

    @Logged
    public boolean isElevatorAtPosition() {
        return Math.abs(targetHeight - getElevatorHeight()) <= ELEVATOR_POSITION_TOLERANCE;
    }

    @Logged
    public boolean isElevatorMoving() {
        return Math.abs(elevatorVelocity.refresh().getValueAsDouble()) >= ELEVATOR_VELOCITY_TOLERANCE;
    }

    @Logged
    public boolean isInSecondStage() {
        return getElevatorHeight() > ELEVATOR_LOW_STAGE_MAX;
    }

    // Arm

    @Logged(name = "armPosition")
    public double getArmPosition() {
        return armPosition.getValueAsDouble();
    }

    @Logged(name = "armTarget")
    public double getArmTarget() {
        return targetAngle;
    }

    @Logged(name = "armVelocity")
    public double getArmVelocity() {
        return armVelocity.getValueAsDouble();
    }

    @Logged(name = "armCurrent")
    public double getArmCurrent() {
        return armCurrent.getValueAsDouble();
    }

    @Logged
    public boolean isArmAtPosition() {
        return Math.abs(getArmPosition() - targetAngle) < ARM_POSITION_TOLERANCE;
    }

    @Logged
    public boolean isArmMoving() {
        return Math.abs(armVelocity.refresh().getValueAsDouble()) >= ARM_VELOCITY_TOLERANCE;
    }

    // -- Control --

    public void setElevatorHeight(double height) {
        if (!elevatorCalibrated) {
            Logger.error("Cannot move elevator without calibration!");
            return;
        }

        targetHeight = MathUtil.clamp(height, ELEVATOR_MIN_POSE, ELEVATOR_MAX_POSE);
        elevatorRightMotor.setControl(
            new Follower(RobotMap.ELEVATOR_LEFT_MOTOR, true)
        );
        elevatorLeftMotor.setControl(
            elevatorPositionControl.withPosition(targetHeight)
        );

        Logger.info("Set elevator height: " + targetHeight);
    }

    public void setState(WindmillState state) {
        setElevatorHeight(state.elevatorState().heightMeters());
        setArmPosition(Units.radiansToRotations(state.armState().angleRadians()));
    }

    public void setArmPosition(double position) {
        if (!armCalibrated) {
            Logger.error("Cannot move arm without calibration!");
            return;
        }

        targetAngle = position;
        armMotor.setControl(armPositionControl.withPosition(targetAngle));

        Logger.info("Set arm position: " + targetAngle);
    }

    public void stopElevator() {
        elevatorLeftMotor.stopMotor();
    }

    public void stopArm() {
        armMotor.stopMotor();
    }

    // -- Periodic --

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            elevatorPosition, elevatorVelocity, elevatorCurrent, armPosition, armVelocity, armCurrent);
    }

    // -- Trajectory Editor --

    public Command runTrajectoryEditorTrajectory() {
        return Commands.deferredProxy(
            () -> networkTablesTrajectory == null ? Commands.none() : new WindmillMoveCommand(networkTablesTrajectory));
    }

    // -- Overrides --

    @Override
    public void onDisabledInit() {
        stopElevator();
        stopArm();
    }

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            // Elevator test
            SysIdTests.characterize(
                "Elevator", this, elevatorLeftMotor,
                Volts.of(0.25).per(Second), Volts.of(1),
                elevatorRightMotor, true
            ),
            // Arm test
            SysIdTests.characterize(
                "Arm", this, armMotor,
                Volts.of(0.25).per(Second), Volts.of(1)
            )
        );
    }
}
