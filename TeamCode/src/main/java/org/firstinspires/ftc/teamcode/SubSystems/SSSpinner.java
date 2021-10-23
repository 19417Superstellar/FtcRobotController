package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Definition of intake Class <BR>
 *
 * SSSpinner of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     <emsp>INTAKE_MOTOR_STATE for telling if intake is running, stopped, or reversing </emsp> <BR>
 *     <emsp>INTAKE_REVERSE_BUTTON_STATE sees if the intake is on or off </emsp> <BR>
 *
 * The functions are as followed: <BR>
 *     <emsp>runIntakeMotor checks if the intake is not running and runs the intake </emsp> <BR>
 *     <emsp>stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets intakeMotorState to INTAKE_MOTOR_STATE.STOPPED </emsp> <BR>
 *     <emsp>reverseIntakeMotor checks if the intake is not reversing, and sets the intake motor to FORWARD, then also
 *     sets intake motor state to REVERSING</emsp> <BR>
 */
public class SSSpinner {

    //TODO: Update code as needed for Subsystem1

    public DcMotor SSSpinnerMotor = null;

    public enum SSSPINNER_MOTOR_STATE {
        STOPPED,
        CLOCKWISE,
        ANTICLOCKWISE
    }

    public SSSPINNER_MOTOR_STATE SSSpinnerMotorState = SSSPINNER_MOTOR_STATE.STOPPED;

    public double SSSpinnerMotorPower1 = 0.25;//0.9;


    public enum SSSPINNER_BUTTON_STATE {
        ON,
        OFF
    }

    public SSSPINNER_BUTTON_STATE SSSpinnerButtonState;

    public SSSpinner(HardwareMap hardwareMap) {
        SSSpinnerMotor = hardwareMap.dcMotor.get("spmotor");

    }

    public void SSSpinner() {

    }

    /**
     * startForwardSSSPinnerMotor checks if the intake is not running and runs the spinner
     */
    public void startClockwiseSSSpinnerMotor() {
        if (SSSpinnerMotorState == SSSPINNER_MOTOR_STATE.CLOCKWISE) {
            runSSSpinnerMotor(DcMotor.Direction.FORWARD, SSSpinnerMotorPower1);
            SSSpinnerMotorState = SSSPINNER_MOTOR_STATE.STOPPED;
        }
    }

    /**
     * stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
     * and sets intakeMotorState to INTAKE_MOTOR_STATE.STOPPED
     */
    public void stopSSSpinnerMotor() {
        if (SSSpinnerMotorState != SSSPINNER_MOTOR_STATE.STOPPED) {
            runSSSpinnerMotor(DcMotor.Direction.FORWARD, 0.0);
            SSSpinnerMotorState = SSSPINNER_MOTOR_STATE.STOPPED;
        }
    }

    /**
     * reverseIntakeMotor checks if the intake is not reversing, and sets the intake motor to FORWARD, then also
     * ets intake motor state to REVERSING
     */
    public void startAntiClockwiseSSSpinnerMotor() {
        if (SSSpinnerMotorState != SSSPINNER_MOTOR_STATE.ANTICLOCKWISE) {
            runSSSpinnerMotor(DcMotor.Direction.REVERSE, SSSpinnerMotorPower1);
            SSSpinnerMotorState = SSSPINNER_MOTOR_STATE.ANTICLOCKWISE;
        }
    }

    private void runSSSpinnerMotor(DcMotor.Direction direction, double power) {
        SSSpinnerMotor.setDirection(direction);
        SSSpinnerMotor.setPower(power);
    }

    //Returns the state of the Spinner Motor
    public SSSpinner.SSSPINNER_MOTOR_STATE getSSSpinnerMotorState() {
        return SSSpinnerMotorState;
    }


}