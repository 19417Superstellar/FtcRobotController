package org.firstinspires.ftc.teamcode.SubSystems;

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

    public Servo bucketServo = null;

    public static final double BUCKET_SERVO_COLLECT_POSITION = 0.87;
    public static final double BUCKET_SERVO_TRANSPORT_POSITION = 0.7;
    public static final double BUCKET_SERVO_DROP_POSITION = 0.40;

    public enum BUCKET_SERVO_STATE {
        COLLECT_POSITION,
        TRANSPORT_POSITION,
        DROP_POSITION
    }

    public BUCKET_SERVO_STATE bucketServoState = BUCKET_SERVO_STATE.COLLECT_POSITION;

    public enum BUCKET_BUTTON_STATE {
        ON,
        OFF
    }
    public BUCKET_BUTTON_STATE bucketButtonState;

    public SSBucket(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.servo.get("bucket_servo");
    }

    public void initBucket(){
        if(bucketServoState != BUCKET_SERVO_STATE.COLLECT_POSITION){
            setToCollect();
        }
    }


    /**
     * set bucket position to collect
     */
    public void setToCollect() {
        if (bucketServoState != BUCKET_SERVO_STATE.COLLECT_POSITION) {
            bucketServo.setPosition(BUCKET_SERVO_COLLECT_POSITION);
            bucketServoState = BUCKET_SERVO_STATE.COLLECT_POSITION;
        }

    }

    /**
     * set bucket position to transport
     */
    public void setToTransport(){
        if (bucketServoState != BUCKET_SERVO_STATE.TRANSPORT_POSITION) {
            bucketServo.setPosition(BUCKET_SERVO_TRANSPORT_POSITION);
            bucketServoState = BUCKET_SERVO_STATE.TRANSPORT_POSITION;
        }
    }

    /**
     * set bucket position to drop
     */
    public void setToDrop(){
        if (bucketServoState != BUCKET_SERVO_STATE.DROP_POSITION) {
            bucketServo.setPosition(BUCKET_SERVO_DROP_POSITION);
            bucketServoState = BUCKET_SERVO_STATE.DROP_POSITION;
        }
    }
    /**
     * returns servo state
     */
    public BUCKET_SERVO_STATE getBucketServoState() {
        return bucketServoState;
    }
}