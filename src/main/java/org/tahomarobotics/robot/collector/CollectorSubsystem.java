package org.tahomarobotics.robot.collector;

import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.collector.CollectorConstants.*;

public class CollectorSubsystem extends AbstractSubsystem {
    // -- Motors --
    private final TalonFX rollerMotor = new TalonFX(RobotMap.COLLECTOR_ROLLER_MOTOR);
    private final TalonFX leftPivotMotor = new TalonFX(RobotMap.COLLECTOR_LEFT_PIVOT_MOTOR);
    private final TalonFX rightPivotMotor = new TalonFX(RobotMap.COLLECTOR_RIGHT_PIVOT_MOTOR);

    // -- Roller motor status signals --
    private final StatusSignal<AngularVelocity> rollerMotorVelocity;
    private final StatusSignal<Voltage> rollerMotorVoltage;
    private final StatusSignal<Current> rollerMotorCurrent;

    // -- Left pivot motor status signals --
    private final StatusSignal<Angle> leftPivotMotorPosition;
    private final StatusSignal<AngularVelocity> leftPivotMotorVelocity;
    private final StatusSignal<Voltage> leftPivotMotorVoltage;
    private final StatusSignal<Current> leftPivotMotorCurrent;

    // -- Right pivot motor status signals --
    private final StatusSignal<Angle> rightPivotMotorPosition;
    private final StatusSignal<AngularVelocity> rightPivotMotorVelocity;
    private final StatusSignal<Voltage> rightPivotMotorVoltage;
    private final StatusSignal<Current> rightPivotMotorCurrent;

    // -- Control requests --
    private final VoltageOut voltageControl = new VoltageOut(0);

    // -- States --
    private RollerMotorState rollerMotorState = RollerMotorState.IDLE;
    private PivotMotorState pivotMotorState = PivotMotorState.STOWED;

    CollectorSubsystem() {
        // Configure motors
        RobustConfigurator.tryConfigureTalonFX("Roller Collector Motor", rollerMotor, createCollectorMotorConfig);
        RobustConfigurator.tryConfigureTalonFX("Left Pivot Collector Motor", leftPivotMotor, createCollectorMotorConfig);
        RobustConfigurator.tryConfigureTalonFX("Right Pivot Collector Motor", rightPivotMotor, createCollectorMotorConfig);

        // Set up left pivot motor to follow right pivot motor
        leftPivotMotor.setControl(new Follower(RobotMap.COLLECTOR_RIGHT_PIVOT_MOTOR, MotorAlignmentValue.Opposed));

        // Bind roller motor status signals
        rollerMotorVelocity = rollerMotor.getVelocity();
        rollerMotorVoltage = rollerMotor.getMotorVoltage();
        rollerMotorCurrent = rollerMotor.getSupplyCurrent();

        // Bind left pivot motor status signals
        leftPivotMotorPosition = leftPivotMotor.getPosition();
        leftPivotMotorVelocity = leftPivotMotor.getVelocity();
        leftPivotMotorVoltage = leftPivotMotor.getMotorVoltage();
        leftPivotMotorCurrent = leftPivotMotor.getSupplyCurrent();

        // Bind right pivot motor status signals
        rightPivotMotorPosition = rightPivotMotor.getPosition();
        rightPivotMotorVelocity = rightPivotMotor.getVelocity();
        rightPivotMotorVoltage = rightPivotMotor.getMotorVoltage();
        rightPivotMotorCurrent = rightPivotMotor.getSupplyCurrent();

        zeroPivotMotors();
    }

    @Override
    public void subsystemPeriodic() {
        BaseStatusSignal.refreshAll(rollerMotorVelocity, rollerMotorVoltage, rollerMotorCurrent,
            leftPivotMotorPosition, leftPivotMotorVelocity, leftPivotMotorVoltage, leftPivotMotorCurrent,
            rightPivotMotorPosition, rightPivotMotorVelocity, rightPivotMotorVoltage, rightPivotMotorCurrent);

        Logger.recordOutput("Collector/Roller Motor/Velocity", rollerMotorVelocity.getValue());
        Logger.recordOutput("Collector/Roller Motor/Voltage", rollerMotorVoltage.getValue());
        Logger.recordOutput("Collector/Roller Motor/Current", rollerMotorCurrent.getValue());

        Logger.recordOutput("Collector/Left Pivot Motor/Position", leftPivotMotorPosition.getValue());
        Logger.recordOutput("Collector/Left Pivot Motor/Velocity", leftPivotMotorVelocity.getValue());
        Logger.recordOutput("Collector/Left Pivot Motor/Voltage", leftPivotMotorVoltage.getValue());
        Logger.recordOutput("Collector/Left Pivot Motor/Current", leftPivotMotorCurrent.getValue());

        Logger.recordOutput("Collector/Right Pivot Motor/Position", rightPivotMotorPosition.getValue());
        Logger.recordOutput("Collector/Right Pivot Motor/Velocity", rightPivotMotorVelocity.getValue());
        Logger.recordOutput("Collector/Right Pivot Motor/Voltage", rightPivotMotorVoltage.getValue());
        Logger.recordOutput("Collector/Right Pivot Motor/Current", rightPivotMotorCurrent.getValue());
    }

    public void zeroPivotMotors() {
        Command zeroCommand = this.runOnce(() -> setPivotMotorVoltage(PIVOT_ZEROING_VOLTAGE))
            .andThen(Commands.waitSeconds(PIVOT_ZEROING_WAIT_SECONDS))
            .andThen(Commands.waitUntil(() -> getPivotMotorVelocity().isNear(PIVOT_ZERO_VELOCITY, PIVOT_ZER0_VELOCITY_THRESHOLD))
            .andThen(() -> {
                rightPivotMotor.setPosition(PIVOT_ZERO_ANGLE);
                leftPivotMotor.setPosition(PIVOT_ZERO_ANGLE);
            })
            .andThen(() -> setPivotMotorVoltage(Volts.of(0)))
        );

        RobotModeTriggers.autonomous()
            .or(RobotModeTriggers.teleop())
            .onTrue(zeroCommand);
    }

    // -- Setters --

    public void setRollerMotorState(RollerMotorState rollerMotorState) {
        this.rollerMotorState = rollerMotorState;

        rollerMotor.setControl(this.rollerMotorState.controlRequest);
    }

    public void setPivotMotorState(PivotMotorState pivotMotorState) {
        this.pivotMotorState = pivotMotorState;

        rightPivotMotor.setControl(this.pivotMotorState.controlRequest);
    }

    public void setPivotMotorVoltage(Voltage voltage) {
        rightPivotMotor.setControl(voltageControl.withOutput(voltage));
    }

    // -- Getters --

    public AngularVelocity getPivotMotorVelocity() {
        return rightPivotMotorVelocity.getValue();
    }

    // -- States --

    enum RollerMotorState {
        COLLECTING(new MotionMagicVelocityVoltage(COLLECTING_VELOCITY), COLLECTING_VELOCITY),
        IDLE(new NeutralOut(), IDLE_VELOCITY),
        EJECTING(new MotionMagicVelocityVoltage(EJECTING_VELOCITY), EJECTING_VELOCITY);

        private final ControlRequest controlRequest;
        private final AngularVelocity velocity;

        RollerMotorState(ControlRequest controlRequest, AngularVelocity velocity) {
            this.controlRequest = controlRequest;
            this.velocity = velocity;
        }
    }

    enum PivotMotorState {
        DEPLOYED(new MotionMagicVoltage(DEPLOYED_ANGLE), DEPLOYED_ANGLE),
        STOWED(new MotionMagicVoltage(STOWED_ANGLE), STOWED_ANGLE),
        ZEROING(new VoltageOut(PIVOT_ZEROING_VOLTAGE), PIVOT_ZERO_ANGLE);

        private final ControlRequest controlRequest;
        private final Angle angle;

        PivotMotorState(ControlRequest controlRequest, Angle angle) {
            this.controlRequest = controlRequest;
            this.angle = angle;
        }
    }
}
