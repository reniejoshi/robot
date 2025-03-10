package org.tahomarobotics.robot.windmill;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.game.GamePiece;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tahomarobotics.robot.windmill.commands.WindmillCommands;
import org.tahomarobotics.robot.windmill.commands.WindmillMoveCommand;
import org.tinylog.Logger;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.windmill.WindmillConstants.*;

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
    private final StatusSignal<Current> elevatorLeftCurrent, elevatorRightCurrent, armCurrent;

    private final LoggedStatusSignal[] statusSignals;

    // Control Requests

    private final MotionMagicVoltage elevatorPositionControl = new MotionMagicVoltage(0);
    private final MotionMagicVoltage armPositionControl = new MotionMagicVoltage(0);

    // State

    private double targetHeight; // Meters
    private double targetAngle;  // Radians
    private double simHeight;
    private double simAngle;
    private double simElevVelocity;
    private double simArmVelocity;
    private WindmillState targetState = WindmillState.fromPrevious(0, 0, 0, null);
    private TrajectoryState targetTrajectoryState = TrajectoryState.STOW;

    @AutoLogOutput(key = "Windmill/Is Zeroed?")
    private boolean zeroed = false;

    // Trajectory

    public final WindmillMechanism windmillMechanism;
    // -- Initialization --

    /**
     *   Arm Angle must be in units of Radians for the Robot.  The CTRE interfaces (control and feedback) are Rotations.  Only convert to and from Radians and
     *   Rotations for those interfaces.
     */
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

        // Status Signals

        elevatorPosition = elevatorLeftMotor.getPosition();
        elevatorVelocity = elevatorLeftMotor.getVelocity();
        elevatorLeftCurrent = elevatorLeftMotor.getSupplyCurrent();
        elevatorRightCurrent = elevatorRightMotor.getSupplyCurrent();

        armPosition = armMotor.getPosition();
        armVelocity = armMotor.getVelocity();
        armCurrent = armMotor.getSupplyCurrent();

        statusSignals = new LoggedStatusSignal[]{
            new LoggedStatusSignal("Elevator Position", elevatorPosition),
            new LoggedStatusSignal("Elevator Velocity", elevatorVelocity),
            new LoggedStatusSignal("Elevator Left Current", elevatorLeftCurrent),
            new LoggedStatusSignal("Elevator Right Current", elevatorRightCurrent),
            new LoggedStatusSignal("Arm Position", armPosition),
            new LoggedStatusSignal("Arm Velocity", armVelocity),
            new LoggedStatusSignal("Arm Current", armCurrent),
            new LoggedStatusSignal("Elevator Encoder Position", elevatorEncoder.getPosition()),
            new LoggedStatusSignal("Elevator Encoder Velocity", elevatorEncoder.getVelocity()),
            new LoggedStatusSignal("Arm Encoder Position", armEncoder.getPosition()),
            new LoggedStatusSignal("Arm Encoder Velocity", armEncoder.getVelocity()),
            new LoggedStatusSignal("Elevator Left Motor Voltage", elevatorLeftMotor.getMotorVoltage()),
            new LoggedStatusSignal("Arm Motor Voltage", armMotor.getMotorVoltage())
        };

        LoggedStatusSignal.setUpdateFrequencyForAll(statusSignals, RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);

        ParentDevice.optimizeBusUtilizationForAll(
            elevatorLeftMotor, elevatorRightMotor, elevatorEncoder, armMotor);

        windmillMechanism = new WindmillMechanism();
    }

    public static Windmill getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        // Publish calibration commands
        SmartDashboard.putData("Zero Windmill (Moving)", WindmillCommands.createElevatorZeroCommand(this));
        SmartDashboard.putData("Calibrate Windmill", WindmillCommands.createCalibrateCommand(this));

        // Calibrate on first enable
        new Trigger(RobotState::isEnabled).onTrue(Commands.runOnce(this::calibrate).onlyIf(() -> !zeroed));
        new Trigger(RobotState::isEnabled).onTrue(Commands.runOnce(this::calibrate).onlyIf(() -> !zeroed));

        // Calibrate with user button
        new Trigger(RobotController::getUserButton).onTrue(WindmillCommands.createUserButtonCalibrateCommand(this));

        return this;
    }

    // -- Calibration --

    public void disableBrakeMode() {
        elevatorLeftMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        armMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    }

    public void calibrate() {
        elevatorEncoder.setPosition(0);
        armEncoder.setPosition(ARM_CALIBRATION_POSE / ARM_BELT_REDUCTION);

        simHeight = TrajectoryState.START.elev;
        simAngle = TrajectoryState.START.arm;
        zeroed = true;
        Logger.info("Windmill Calibrated");
        enableBrakeMode();
        setTargetState(TrajectoryState.STOW);
    }

    public void enableBrakeMode() {
        elevatorLeftMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        armMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    public void setElevatorVoltage(double volts) {
        elevatorLeftMotor.setVoltage(volts);
    }

    // -- Getters --

    public WindmillState getCurrentState() {
        WindmillState.ElevatorState elevatorState = new WindmillState.ElevatorState(getElevatorHeight(), getElevatorVelocity(), getElevatorAcceleration());

        WindmillState.ArmState armState = new WindmillState.ArmState(getArmPosition(), getArmVelocity(), getArmAcceleration());

        return new WindmillState(0, elevatorState, armState);
    }

    @AutoLogOutput(key = "Windmill/Target State")
    public WindmillState getTargetState() {
        return targetState;
    }

    @AutoLogOutput(key = "Windmill/Target Trajectory State")
    public TrajectoryState getTargetTrajectoryState() {
        return targetTrajectoryState;
    }

    public boolean isZeroed() {
        return zeroed;
    }

    public boolean isScoringCoral() {
        return !(targetTrajectoryState == WindmillConstants.TrajectoryState.COLLECT || targetTrajectoryState == WindmillConstants.TrajectoryState.STOW) &&
               Collector.getInstance().getCollectionMode() != GamePiece.ALGAE;
    }

    // Elevator

    @AutoLogOutput(key = "Windmill/Elevator/Height")
    public double getElevatorHeight() {
        return Robot.isReal() ? elevatorPosition.getValueAsDouble() : simHeight;
    }

    private double getElevatorVelocity() {
        return Robot.isReal() ? elevatorVelocity.getValueAsDouble() : simElevVelocity;
    }

    private double getElevatorAcceleration() {
        return elevatorLeftMotor.getAcceleration().refresh().getValueAsDouble();
    }

    @AutoLogOutput(key = "Windmill/Elevator/Target Height")
    public double getElevatorTarget() {
        return targetHeight;
    }

    @AutoLogOutput(key = "Windmill/Elevator/Is at Position?")
    public boolean isElevatorAtPosition() {
        return Math.abs(getElevatorTarget() - getElevatorHeight()) < ELEVATOR_POSITION_TOLERANCE;
    }

    @AutoLogOutput(key = "Windmill/Elevator/Is Moving?")
    public boolean isElevatorMoving() {
        return Math.abs(getElevatorVelocity()) > ELEVATOR_VELOCITY_TOLERANCE;
    }

    public double getElevatorLeftCurrent() {
        return elevatorLeftCurrent.getValueAsDouble();
    }

    public double getElevatorRightCurrent() {
        return elevatorRightCurrent.getValueAsDouble();
    }

    // Arm

    @AutoLogOutput(key = "Windmill/Arm/Position")
    public double getArmPosition() {
        return Robot.isReal() ? Units.rotationsToRadians(armPosition.getValueAsDouble()) : simAngle;
    }

    public double getArmVelocity() {
        return Robot.isReal() ? Units.rotationsToRadians(armVelocity.refresh().getValueAsDouble()) : simArmVelocity;
    }

    private double getArmAcceleration() {
        return Units.rotationsToRadians(armMotor.getAcceleration().refresh().getValueAsDouble());
    }

    @AutoLogOutput(key = "Windmill/Arm/Target Position")
    private double getArmTarget() {
        return targetAngle;
    }

    @AutoLogOutput(key = "Windmill/Arm/Is at Position?")
    public boolean isArmAtPosition() {
        return Math.abs(getArmPosition() - getArmTarget()) < ARM_POSITION_TOLERANCE;
    }

    @AutoLogOutput(key = "Windmill/Arm/Is Moving?")
    public boolean isArmMoving() {
        return Math.abs(getArmVelocity()) >= ARM_VELOCITY_TOLERANCE;
    }

    @AutoLogOutput(key = "Windmill/Is at Target Trajectory State?")
    public boolean isAtTargetTrajectoryState() {
        return (Math.abs(getElevatorHeight() - targetTrajectoryState.elev) < ELEVATOR_POSITION_TOLERANCE) &&
               (Math.abs(getArmPosition() - targetTrajectoryState.arm) < ARM_POSITION_TOLERANCE);

    }

    public double getArmCurrent() {
        return armCurrent.getValueAsDouble();
    }

    // -- Control --

    public void setTargetState(TrajectoryState targetState) {
        this.targetTrajectoryState = targetState;
    }

    public void setState(WindmillState state) {
        targetState = state;
        windmillMechanism.update(state.elevatorState().heightMeters(), state.armState().angleRadians());

        setElevatorHeight(state.elevatorState().heightMeters());
        setArmPosition(state.armState().angleRadians());
        simElevVelocity = state.elevatorState().velocityMetersPerSecond() * 0.95;
        simArmVelocity = state.armState().velocityRadiansPerSecond() * 0.95;
    }

    public void setElevatorHeight(double height) {
        if (!zeroed) {
            Logger.error("Cannot move elevator without calibration!");
            return;
        }
        targetHeight = MathUtil.clamp(height, ELEVATOR_MIN_POSE, ELEVATOR_MAX_POSE);
        elevatorRightMotor.setControl(new Follower(RobotMap.ELEVATOR_LEFT_MOTOR, true));
        elevatorLeftMotor.setControl(elevatorPositionControl.withPosition(targetHeight));
        simHeight = height;
    }

    public void setArmPosition(double position) {
        if (!zeroed) {
            Logger.error("Cannot move arm without calibration!");
            return;
        }

        targetAngle = position;
        armMotor.setControl(armPositionControl.withPosition(Units.radiansToRotations(targetAngle)));
        simAngle = position;
    }

    public Command createSyncCollectionModeCommand() {
        return Commands.deferredProxy(() -> {
            if (Collector.getInstance().getCollectionMode() == GamePiece.CORAL) {
                if (targetTrajectoryState == TrajectoryState.L3) {
                    return createTransitionCommand(TrajectoryState.HIGH_DESCORE);
                } else if (targetTrajectoryState == TrajectoryState.L2) {
                    return createTransitionCommand(TrajectoryState.LOW_DESCORE);
                }
            } else {
                if (targetTrajectoryState == TrajectoryState.HIGH_DESCORE) {
                    return createTransitionCommand(TrajectoryState.L3);
                } else if (targetTrajectoryState == TrajectoryState.LOW_DESCORE) {
                    return createTransitionCommand(TrajectoryState.L2);
                }
            }
            return Commands.none();
        });
    }

    public Command createTransitionCommand(TrajectoryState to) {
        return Commands.deferredProxy(() -> {
            // Default to COLLECT if at target state (i.e. L4 -x> L4 -> COLLECT)
            TrajectoryState target = targetTrajectoryState == to ? TrajectoryState.COLLECT : to;

            Optional<Command> output = WindmillMoveCommand.fromTo(targetTrajectoryState, target);
            if (output.isEmpty()) {
                output = WindmillMoveCommand.fromTo(targetTrajectoryState, TrajectoryState.STOW);
                Logger.error("WindmillMoveCommand from " + targetTrajectoryState + " to " + target + " failed.");
            }
            return output.orElseGet(Commands::none);
        });
    }

    public Command createTransitionToggleCommand(TrajectoryState onTrue, TrajectoryState onFalse) {
        return Commands.deferredProxy(() -> {
            if (targetTrajectoryState == onFalse) {
                Logger.info("Toggling between {} and {}", onFalse, onTrue);
                return createTransitionCommand(onTrue);
            } else if (targetTrajectoryState == onTrue) {
                Logger.info("Toggling between {} and {}", onTrue, onFalse);
                return createTransitionCommand(onFalse);
            } else {
                return Commands.runOnce(() -> Logger.error("Cannot toggle from an untoggleable state."));
            }
        });
    }

    public Command createResetToPreviousState() {
        return Commands.runOnce(
            () -> {
                Logger.info("Windmill attempting to go to " + targetTrajectoryState);
                setState(targetTrajectoryState.state);
            }, this
        ).withName("Windmill - Reset to Target State");
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
        LoggedStatusSignal.refreshAll(statusSignals);
        LoggedStatusSignal.log("Windmill/", statusSignals);

    }

    // -- Overrides --

    @Override
    public void onTeleopInit() {
        Commands.waitUntil(() -> zeroed)
                .andThen(createResetToPreviousState()).schedule();
    }

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

    private class WindmillMechanism {
        private final MechanismLigament2d elevator;
        private final MechanismLigament2d arm;
        private static final double ELEV_OFFSET = Units.inchesToMeters(24);
        private double prevHeight = 0;
        private double prevAngle = 0;

        public WindmillMechanism() {
            Mechanism2d mech = new Mechanism2d(3, 3);
            MechanismRoot2d root = mech.getRoot("climber", 2, 0);
            elevator = root.append(new MechanismLigament2d("elevator", ELEV_OFFSET, 90));
            arm = elevator.append(new MechanismLigament2d("wrist", ARM_LENGTH, 90, 6, new Color8Bit(Color.kPurple)));

            SmartDashboard.putData("Windmill", mech);
        }

        public void update(double evelHeight, double armAngle) {
            if (evelHeight != prevHeight || armAngle != prevAngle) {
                prevHeight = evelHeight;
                prevAngle = armAngle;
                elevator.setLength(ELEV_OFFSET + evelHeight);
                arm.setAngle(Units.radiansToDegrees(armAngle) - 90);

                Logger.info(String.format("Windmill; %7.2f %7.2f", Units.metersToInches(evelHeight), Units.radiansToDegrees(armAngle)));
            }
        }
    }

}
