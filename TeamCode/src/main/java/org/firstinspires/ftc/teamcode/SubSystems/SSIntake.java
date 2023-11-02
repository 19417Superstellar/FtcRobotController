package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Definition of SSIntake Class <BR>
 *
 * HzIntake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
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
public class SSIntake {

    public DcMotorEx intakeMotor;

    public enum SSINTAKE_MOTOR_STATE {
        RUNNING,
        STOPPED,
        REVERSING
    }

    public SSINTAKE_MOTOR_STATE SSIntakeMotorState = SSINTAKE_MOTOR_STATE.RUNNING;

    public double SSIntakeMotorPower = 0.95;//0.9;
    // set intake to eject once distance activated
    //
    public enum SSINTAKE_BUTTON_STATE {
        ON,
        OFF
    }
    public SSINTAKE_BUTTON_STATE SSIntakeButtonState;
    public Telemetry telemetry;
    public SSIntake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        initSSIntake();
    }

    public void initSSIntake(){

    }

    /**
     * runIntakeMotor checks if the intake is not running and runs the intake
     */
    public void startForwardSSIntakeMotor() {
        if(SSIntakeMotorState != SSINTAKE_MOTOR_STATE.RUNNING) {
            runSSIntakeMotor(DcMotor.Direction.FORWARD,SSIntakeMotorPower);
            SSIntakeMotorState = SSINTAKE_MOTOR_STATE.RUNNING;
        }
    }

    /**
     * stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
     * and sets intakeMotorState to INTAKE_MOTOR_STATE.STOPPED
     */
    public void stopSSIntakeMotor() {
        if(SSIntakeMotorState != SSINTAKE_MOTOR_STATE.STOPPED) {
            runSSIntakeMotor(DcMotor.Direction.FORWARD, 0.0);
            SSIntakeMotorState = SSINTAKE_MOTOR_STATE.STOPPED;
        }
    }

    /**
     * reverseIntakeMotor checks if the intake is not reversing, and sets the intake motor to FORWARD, then also
     * ets intake motor state to REVERSING
     */
    public void startReverseSSIntakeMotor() {
        if(SSIntakeMotorState != SSINTAKE_MOTOR_STATE.REVERSING) {
            runSSIntakeMotor(DcMotor.Direction.REVERSE, SSIntakeMotorPower);
            SSIntakeMotorState = SSINTAKE_MOTOR_STATE.REVERSING;

        }
    }

    /**
     * Run SS Intake motor
     */
    public void runSSIntakeMotor(DcMotor.Direction direction, double power){
        intakeMotor.setDirection(direction);
        intakeMotor.setPower(power);
    }

    /**
     * Returns Intake motor state
     */
    public SSINTAKE_MOTOR_STATE getSSIntakeMotorState() {
        return SSIntakeMotorState;
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("=============");
    }
}