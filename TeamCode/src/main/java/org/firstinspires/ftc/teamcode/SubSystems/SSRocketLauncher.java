package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SSRocketLauncher {

    public Servo launcherServo;

    //Gobilda (enter motor details later
    //Encoder count : enter details to be done


    public enum LAUNCHER_SERVO_STATE {
        LAUNCHER_OPEN,
        LAUNCHER_CLOSE
    }

    public LAUNCHER_SERVO_STATE launcherServoState = LAUNCHER_SERVO_STATE.LAUNCHER_OPEN;

    // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
    //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Encoder Resolution: 537.7 PPR at the Output Shaft
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/

    //MAX 2200

    //change values
    public static double LAUNCHER_OPEN_POSITION = 0.4;
    public static double LAUNCHER_CLOSE_POSITION = 0.6;

    public Telemetry telemetry;
    public SSRocketLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        launcherServo = hardwareMap.get(Servo.class,"launcher_servo");
        initLauncher();
    }

    /**
     * Initialization for the Arm
     */
    public void initLauncher(){ //TODO Amjad : If the starting position in autonomous is with arm facing forward, arm may need to be kept open to git in 18"
        launcherServo.setPosition(LAUNCHER_CLOSE_POSITION);
        launcherServoState = LAUNCHER_SERVO_STATE.LAUNCHER_CLOSE;
    }


    public void setLauncherOpen() {
        launcherServo.setPosition(LAUNCHER_OPEN_POSITION);
        launcherServoState = LAUNCHER_SERVO_STATE.LAUNCHER_OPEN;
    }


    public void setLauncherClose() {
        launcherServo.setPosition(LAUNCHER_CLOSE_POSITION);
        launcherServoState = LAUNCHER_SERVO_STATE.LAUNCHER_CLOSE;
    }


    //function to determine POWER_WITH_CARGO or POWER_WITH_NO_CARGO


    public LAUNCHER_SERVO_STATE getGripServoState() {
        return launcherServoState;
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("=============");
    }

}