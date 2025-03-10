package org.tahomarobotics.robot.windmill;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Dynamics {

    private record State(double position, double velocity, double acceleration) {}

    private record Voltages(double elevatorVoltage, double armVoltage) {}

    private static final double g = 9.80665;
    private static final double me = Units.lbsToKilograms(7.9);
    private static final double me_2nd = Units.lbsToKilograms(5.9);
    private static final double ma = Units.lbsToKilograms(6.6);
    private static final double lc = Units.inchesToMeters(14.838);
    private static final double I = Units.lbsToKilograms(804.076)*Units.inchesToMeters(1.0)*Units.inchesToMeters(1.0);
    private static final double coral = Units.lbsToKilograms(1);

    private double kElev = WindmillConstants.ELEVATOR_GEAR_REDUCTION * WindmillConstants.ELEVATOR_MAIN_PULLEY_RADIUS;
    private double kArm = WindmillConstants.ARM_GEAR_REDUCTION;

    private DCMotor elevatorMotors = DCMotor.getKrakenX60Foc(2);
    private DCMotor armMotor = DCMotor.getKrakenX60Foc(1);

    private static final double SECOND_STAGE_THRESHHOLD = Units.inchesToMeters(12);

    // ma, lc and I will change with coral
    private enum K {
        EMPTY_1ST_STAGE(me + ma,ma * lc,ma * lc * lc + I),
        EMPTY_2ND_STAGE(me + ma + me_2nd,ma * lc,ma * lc * lc + I),
        CORAL_1ST_STAGE(me + ma + coral,ma * lc,ma * lc * lc + I),
        CORAL_2ND_STAGE(me + ma + me_2nd + coral,ma * lc,ma * lc * lc + I);

        final double  me_ma;
        final double ma_lc;
        final double ma_lc2_I;
        K(double me_ma, double ma_lc, double ma_lc2_I) {
            this.me_ma = me_ma;
            this.ma_lc = ma_lc;
            this.ma_lc2_I = ma_lc2_I;
        }
    }


    public Voltages inverseDynamics(State elevatorState, State armState, double robotAcceleration, boolean hasCoral) {

        double sinTheta = Math.sin(armState.position);
        double cosTheta = Math.cos(armState.position);

        // select constants base on elevatorState.position and if coral is attached
        K k = elevatorState.position < SECOND_STAGE_THRESHHOLD ?
            ( hasCoral ? K.CORAL_1ST_STAGE : K.EMPTY_1ST_STAGE ) : (hasCoral ? K.CORAL_2ND_STAGE : K.EMPTY_2ND_STAGE);

        // forces due to acceleration
        double elevatorForce =
            k.me_ma * elevatorState.acceleration +
            k.ma_lc * cosTheta * armState.acceleration;
        double armTorque =
            -k.ma_lc * sinTheta * robotAcceleration +
            k.ma_lc * cosTheta * elevatorState.acceleration +
            k.ma_lc2_I * armState.acceleration;

        // forces due to velocity
        elevatorForce += -k.ma_lc * armState.velocity * armState.velocity * sinTheta;

        // forces due to gravity
        elevatorForce += k.me_ma * g;
        armTorque += k.ma_lc * g * cosTheta;

        // motor torques
        double elevatorMotorTorque = elevatorForce * kElev;
        double armMotorTorque = armTorque * kArm;

        // motor velocities
        double elevatorMotorVelocity = elevatorState.velocity / kElev;
        double armMotorVelocity = armState.velocity / kArm;

        Voltages voltages = new Voltages(
            elevatorMotors.getVoltage(elevatorMotorTorque, elevatorMotorVelocity),
            armMotor.getVoltage(armMotorTorque, armMotorVelocity));

        return voltages;
    }
}
