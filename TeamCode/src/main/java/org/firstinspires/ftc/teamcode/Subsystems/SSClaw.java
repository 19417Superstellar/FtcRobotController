package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SSClaw {

    public Servo gripServoRight;
    public Servo gripServoLeft;
    //Gobilda (enter motor details later
    //Encoder count : enter details to be done


    public enum GRIP_SERVO_STATE {
        GRIP_OPEN,
        GRIP_CLOSE
    }

    public GRIP_SERVO_STATE gripServoState = GRIP_SERVO_STATE.GRIP_OPEN;

    // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
    //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Encoder Resolution: 537.7 PPR at the Output Shaft
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/

    //MAX 2200

    public static double GRIP_OPEN_POSITION_LEFT = 0;
    public static double GRIP_CLOSE_POSITION_LEFT = 0.25;

    public static double GRIP_OPEN_POSITION_RIGHT = 1.0;
    public static double GRIP_CLOSE_POSITION_RIGHT = 0.75;



    public SSClaw(HardwareMap hardwareMap) {
        gripServoRight = hardwareMap.get(Servo.class,"grip_servo_right");
        gripServoLeft = hardwareMap.get(Servo.class,"grip_servo_left");

        initGrip();
    }

    /**
     * Initialization for the Arm
     */
    public void initGrip(){ //TODO Amjad : If the starting position in autonomous is with arm facing forward, arm may need to be kept open to git in 18"
        gripServoLeft.setPosition(GRIP_CLOSE_POSITION_LEFT);
        gripServoRight.setPosition(GRIP_CLOSE_POSITION_RIGHT);
        gripServoState = GRIP_SERVO_STATE.GRIP_CLOSE;
    }


    public void setGripOpen() {
            gripServoRight.setPosition(GRIP_OPEN_POSITION_RIGHT);
            gripServoLeft.setPosition(GRIP_OPEN_POSITION_LEFT);
          gripServoState = GRIP_SERVO_STATE.GRIP_OPEN;
    }

    public void setGripClose() {
            gripServoRight.setPosition(GRIP_CLOSE_POSITION_RIGHT);
            gripServoLeft.setPosition(GRIP_CLOSE_POSITION_LEFT);
            gripServoState = GRIP_SERVO_STATE.GRIP_CLOSE;
    }

    //function to determinie POWER_WITH_CARGO or POWER_WITH_NO_CARGO


    public GRIP_SERVO_STATE getGripServoState() {
        return gripServoState;
    }


}
