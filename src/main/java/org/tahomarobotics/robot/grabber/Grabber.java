package org.tahomarobotics.robot.grabber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;

import java.util.List;

import static org.tahomarobotics.robot.grabber.GrabberConstants.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Grabber extends SubsystemIF {
    private final static Grabber INSTANCE = new Grabber();

    // -- Member Variables --

    // Hardware

    private final TalonFX grabberMotor;

    // Status Signals

    private final StatusSignal<AngularVelocity> grabberVelocity;
    private final StatusSignal<Current> grabberCurrent;

    @Logged
    private final LoggedStatusSignal.List statusSignals;

    // Control requests

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0).withEnableFOC(
        RobotConfiguration.RIO_PHOENIX_PRO);
    private final VoltageOut voltageControl = new VoltageOut(0);

    // State

    @Logged
    private GrabberState grabberState = GrabberState.DISABLED;
    @Logged
    private int frameCounter = 0;

    // -- Initialization --

    private Grabber() {
        // Create hardware

        grabberMotor = new TalonFX(RobotMap.END_EFFECTOR_MOTOR);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Grabber Motor", grabberMotor, grabberMotorConfig);

        // Bind status signals

        grabberVelocity = grabberMotor.getVelocity();
        grabberCurrent = grabberMotor.getSupplyCurrent();

        statusSignals = new LoggedStatusSignal.List(List.of(
            new LoggedStatusSignal("Grabber Velocity", grabberVelocity),
            new LoggedStatusSignal("Grabber Current", grabberCurrent)
        ));

        statusSignals.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);
        ParentDevice.optimizeBusUtilizationForAll(grabberMotor);
    }

    public static Grabber getInstance() {
        return INSTANCE;
    }

    // -- Getters --

    public GrabberState getGrabberState() {
        return grabberState;
    }

    @Logged(name = "grabberVelocity")
    public double getVelocity() {
        return grabberVelocity.getValueAsDouble();
    }

    // -- Grabber Control --

    public void setGrabberState(GrabberState state) {
        grabberState = state;
    }

    private void ejectCoral() {
        grabberMotor.setControl(velocityControl.withVelocity(GrabberConstants.CORAL_EJECT_VELOCITY));
    }

    private void holdCoral() {
        // Holding must be a voltage because we will not see any feedback for a PID loop
        grabberMotor.setControl(voltageControl.withOutput(GrabberConstants.CORAL_HOLD_VOLTAGE));
    }

    private void collectCoral() {
        grabberMotor.setControl(velocityControl.withVelocity(GrabberConstants.CORAL_COLLECT_VELOCITY));
    }

    private void holdAlgae() {
        grabberMotor.setVoltage(GrabberConstants.ALGAE_HOLD_VOLTAGE);
    }

    private void collectAlgae() {
        grabberMotor.setControl(velocityControl.withVelocity(GrabberConstants.ALGAE_COLLECT_VELOCITY));
    }

    private void ejectAlgae() {
        grabberMotor.setControl(velocityControl.withVelocity(GrabberConstants.ALGAE_EJECT_VELOCITY));
    }

    // -- Periodic --

    @Override
    public void periodic() {
        switch(grabberState) {
            case COLLECTING_CORAL -> {
                collectCoral();
                if (Math.abs(grabberMotor.getVelocity().getValueAsDouble()) < GrabberConstants.CORAL_GRABBED_VELOCITY_THRESHOLD) {
                    frameCounter += 1;
                } else {
                    frameCounter = 0;
                }

                if (frameCounter > GrabberConstants.CORAL_GRABBED_CHECK_TIME / 0.02) {
                    setGrabberState(GrabberState.HOLDING_CORAL);
                    frameCounter = 0;
                }
             }
            case HOLDING_CORAL -> {
                holdCoral();
            }
            case EJECTING_CORAL -> {
                ejectCoral();
                if (Math.abs(grabberMotor.getVelocity().getValueAsDouble()) > GrabberConstants.CORAL_EJECTED_VELOCITY_THRESHOLD) {
                    frameCounter += 1;
                } else {
                    frameCounter = 0;
                }

                if (frameCounter > GrabberConstants.CORAL_EJECTED_CHECK_TIME / 0.02) {
                    setGrabberState(GrabberState.DISABLED);
                    frameCounter = 0;
                }
            }
            case DISABLED ->  {
                grabberMotor.stopMotor();
            }
            case COLLECTING_ALGAE -> {
                collectAlgae();
                if (Math.abs(grabberMotor.getVelocity().getValueAsDouble()) > GrabberConstants.ALGAE_COLLECT_VELOCITY_THRESHOLD) {
                    frameCounter += 1;
                }
                else {
                    frameCounter= 0;
                }

                if (frameCounter > GrabberConstants.ALGAE_COLLECT_CHECK_TIME / 0.02) {
                    setGrabberState(GrabberState.HOLDING_ALGAE);
                    frameCounter = 0;
                }
            }
            case HOLDING_ALGAE -> {
                holdAlgae();
            }
            case EJECTING_ALGAE -> {
                ejectAlgae();
                if (Math.abs(grabberMotor.getVelocity().getValueAsDouble()) > GrabberConstants.ALGAE_EJECT_VELOCITY_THRESHOLD) {
                    frameCounter += 1;
                }
                else {
                    frameCounter = 0;
                }
                if (frameCounter > GrabberConstants.ALGAE_EJECT_CHECK_TIME / 0.02) {
                    setGrabberState(GrabberState.DISABLED);
                    frameCounter = 0;
                }
            }

        }
    }

    // -- States --

    public enum GrabberState {
        DISABLED,
        COLLECTING_CORAL,
        HOLDING_CORAL,
        EJECTING_CORAL,
        COLLECTING_ALGAE,
        EJECTING_ALGAE,
        HOLDING_ALGAE,
    }
}