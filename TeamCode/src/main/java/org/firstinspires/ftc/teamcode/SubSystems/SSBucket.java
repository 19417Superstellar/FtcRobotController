package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SSBucket {

    public Servo bucketServo;


    

    public enum BUCKET_SERVO_STATE {
        BUCKET_PICK,
        BUCKET_CARRY,
        BUCKET_DROP
    }

    public BUCKET_SERVO_STATE bucketServoState = BUCKET_SERVO_STATE.BUCKET_PICK;

    // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
    //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Encoder Resolution: 537.7 PPR at the Output Shaft
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/

    //MAX 2200

    //change values

   // public static double BUCKET_DROP_POSITION = 180;
   // public static double BUCKET_PICK_POSITION = 0;

    public Telemetry telemetry;
    public SSBucket(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        bucketServo = hardwareMap.get(Servo.class,"bucket_servo");

        initBucket();
    }

    /**
     * Initialization for the Bucket
     */

    public void initBucket(){ //TODO Amjad : If the starting position in autonomous is with arm facing forward, arm may need to be kept open to git in 18"
        bucketServo.setPosition(0.00);
        bucketServoState = BUCKET_SERVO_STATE.BUCKET_PICK;
    }
    

    public void setBucketDropPosition() {
        bucketServo.setPosition(0.86);
        bucketServoState = BUCKET_SERVO_STATE.BUCKET_DROP;
    }

    public void setBucketPickPosition() {
        bucketServo.setPosition(0.00);
        bucketServoState = BUCKET_SERVO_STATE.BUCKET_PICK;
    }

    public void setBucketCarryPosition() {
        bucketServo.setPosition(0.42);
        bucketServoState = BUCKET_SERVO_STATE.BUCKET_CARRY;
    }

    //function to determine POWER_WITH_CARGO or POWER_WITH_NO_CARGO



    // both below are placeholders until we can test and get values

    public BUCKET_SERVO_STATE getBucketServoState() {
        return bucketServoState;
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx)
        telemetry.addLine("Bucket: " + getBucketServoState().toString() + "  " + bucketServo.getPosition());
        telemetry.addLine("=============");

    }

}