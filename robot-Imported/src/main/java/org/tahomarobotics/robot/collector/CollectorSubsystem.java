package org.tahomarobotics.robot.collector;

import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.RobustConfigurator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

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
}
