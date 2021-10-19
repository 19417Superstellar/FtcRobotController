package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Definition of HzIntake Class <BR>
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
public class SSBucket {

    //TODO: Update code as needed for Subsystem1

    public Servo bucketServo = null;

    public static final double collectPosition = 0;
    public static final double transportPosition = 0.35;
    public static final double dropPosition = 9;

    public enum BUCKET_SERVO_STATE {
        BUCKET_COLLECT_POSITION,
        BUCKET_TRANSPORT_POSITION,
        BUCKET_DROP_POSITION
    }

    public BUCKET_SERVO_STATE bucketServoState = BUCKET_SERVO_STATE.BUCKET_COLLECT_POSITION;
    public enum BUCKET_BUTTON_STATE {
        ON,
        OFF
    }
    public BUCKET_BUTTON_STATE bucketButtonState;

    public SSBucket(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.servo.get("bucket_servo");
    }

    public void initBucket(){

    }


    /**
     * set bucket position to collect
     */
    public void setToCollect() {
        if (bucketServoState != BUCKET_SERVO_STATE.BUCKET_COLLECT_POSITION) {
            bucketServo.setPosition(collectPosition);

            bucketServoState = BUCKET_SERVO_STATE.BUCKET_COLLECT_POSITION;
        }

    }
    /**
     * set bucket position to transport
     */
    public void setToTransport(){
        if (bucketServoState != BUCKET_SERVO_STATE.BUCKET_TRANSPORT_POSITION) {
            bucketServo.setPosition(transportPosition);

            bucketServoState = BUCKET_SERVO_STATE.BUCKET_TRANSPORT_POSITION;
        }
    }

    /**
     * set bucket position to drop
     */
    public void setToDrop(){
        if (bucketServoState != BUCKET_SERVO_STATE.BUCKET_DROP_POSITION) {
            bucketServo.setPosition(dropPosition);

            bucketServoState = BUCKET_SERVO_STATE.BUCKET_DROP_POSITION;
        }
    }
    /**
     * returns servo state
     */
    public BUCKET_SERVO_STATE getSubsystemServoState() {
        return bucketServoState;
    }
}