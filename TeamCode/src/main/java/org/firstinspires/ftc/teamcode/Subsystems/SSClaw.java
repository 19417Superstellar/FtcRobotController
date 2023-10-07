package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SSClaw {

    public Servo gripServo;
    public Servo wristServo;
    //Gobilda (enter motor details later
    //Encoder count : enter details to be done


    public enum GRIP_SERVO_STATE {
        GRIP_OPEN,
        GRIP_CLOSE
    }

    public enum WRIST_SERVO_STATE {
        WRIST_FORWARD,
        WRIST_BACK
    }

    public GRIP_SERVO_STATE gripServoState = GRIP_SERVO_STATE.GRIP_OPEN;
    public WRIST_SERVO_STATE wristServoState = WRIST_SERVO_STATE.WRIST_FORWARD;

    // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
    //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Encoder Resolution: 537.7 PPR at the Output Shaft
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/

    //MAX 2200

    //change values
    public static double GRIP_OPEN_POSITION = 0;
    public static double GRIP_CLOSE_POSITION = 0.25;

    public static double WRIST_FORWARD_POSITION = 180;
    public static double WRIST_BACK_POSITION= 0;


    public SSClaw(HardwareMap hardwareMap) {
        gripServo = hardwareMap.get(Servo.class,"grip_servo");
        wristServo = hardwareMap.get(Servo.class,"wrist_servo");

        initGrip();
        initWrist();
    }

    /**
     * Initialization for the Arm
     */
    public void initGrip(){ //TODO Amjad : If the starting position in autonomous is with arm facing forward, arm may need to be kept open to git in 18"
        gripServo.setPosition(GRIP_CLOSE_POSITION);
        gripServoState = GRIP_SERVO_STATE.GRIP_CLOSE;
    }

    public void initWrist(){ //TODO Amjad : If the starting position in autonomous is with arm facing forward, arm may need to be kept open to git in 18"
        wristServo.setPosition(WRIST_FORWARD_POSITION);
        wristServoState = WRIST_SERVO_STATE.WRIST_FORWARD;
    }

    public void setGripOpen() {
            gripServo.setPosition(GRIP_OPEN_POSITION);
          gripServoState = GRIP_SERVO_STATE.GRIP_OPEN;
    }

    public void setWristForward() {
        wristServo.setPosition(WRIST_FORWARD_POSITION);
        wristServoState = WRIST_SERVO_STATE.WRIST_FORWARD;
    }

    public void setGripClose() {
            gripServo.setPosition(GRIP_CLOSE_POSITION);
            gripServoState = GRIP_SERVO_STATE.GRIP_CLOSE;
    }

    public void setWristBack() {
        wristServo.setPosition(WRIST_BACK_POSITION);
        wristServoState = WRIST_SERVO_STATE.WRIST_BACK;
    }

    //function to determine POWER_WITH_CARGO or POWER_WITH_NO_CARGO


    public GRIP_SERVO_STATE getGripServoState() {
        return gripServoState;
    }
    public WRIST_SERVO_STATE getWristServoState() {
        return wristServoState;
    }


}
