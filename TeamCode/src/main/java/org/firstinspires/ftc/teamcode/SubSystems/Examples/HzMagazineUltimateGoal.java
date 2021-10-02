package org.firstinspires.ftc.teamcode.SubSystems.Examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Class Description of the Magazine Subsystem
 *
 * Magazine collects the rings and moves up and down to the launcher on an elevator and is
 * controlled by a continuous servomotor.
 */
public class HzMagazineUltimateGoal {
    public CRServo magazineServo;

    public TouchSensor magazineLaunchTouchSensor;
    public TouchSensor magazineCollectTouchSensor;

    LinearOpMode opModepassed;

    public enum MAGAZINE_POSITION {
        AT_COLLECT,
        AT_LAUNCH,
        AT_ERROR
    }

    public enum MOVE_MAGAZINE_TO {
        COLLECT,
        LAUNCH
    }
    public MOVE_MAGAZINE_TO moveMagazineTo = MOVE_MAGAZINE_TO.COLLECT;

    public MAGAZINE_POSITION magazinePosition = MAGAZINE_POSITION.AT_COLLECT;
    public boolean magazinePositionError = false;

    public enum MAGAZINE_TOUCH_SENSORS_STATE {
        LAUNCH_TS_PRESSED,
        LAUNCH_TS_NOT_PRESSED,
        COLLECT_TS_PRESSED,
        COLLECT_TS_NOT_PRESSED,
        TS_ERROR
    }

    public HzMagazineUltimateGoal(HardwareMap hardwareMap) {
       // magazineServo = hardwareMap.crservo.get("mgz_servo");
       // magazineLaunchTouchSensor = hardwareMap.touchSensor.get("mgz_launch_ts");
       // magazineCollectTouchSensor = hardwareMap.touchSensor.get("mgz_collect_ts");

    }

    /**
     * Initialize Magazine
     */
    public void initMagazine(){
        senseMagazinePosition();
        switch (magazinePosition){
            case AT_LAUNCH:
                moveMagazineTo = MOVE_MAGAZINE_TO.LAUNCH;
                moveMagazineToLaunch();
                break;
            case AT_COLLECT:
            case AT_ERROR:
                moveMagazineTo = MOVE_MAGAZINE_TO.COLLECT;
                moveMagazineToCollect();
                break;
        }
    }

    /**
     * Sense the position of magazine based on touch sensor.
     * if in between states mark as error
     */
    public void senseMagazinePosition(){
        MAGAZINE_TOUCH_SENSORS_STATE magazine_touch_sensors_state = getMagazineTouchSensorsState();
        if (magazine_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.LAUNCH_TS_PRESSED) {
            magazinePosition = MAGAZINE_POSITION.AT_LAUNCH;
            magazinePositionError = false;
        } else if (magazine_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_PRESSED) {
            magazinePosition = MAGAZINE_POSITION.AT_COLLECT;
            magazinePositionError = false;
        } else if (magazine_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.TS_ERROR){
            magazinePositionError = true;
        }
    }

    /**
     * Find statr of magazine touch sensors
     */
    public MAGAZINE_TOUCH_SENSORS_STATE getMagazineTouchSensorsState(){
        MAGAZINE_TOUCH_SENSORS_STATE magazine_touch_sensors_state;
        MAGAZINE_TOUCH_SENSORS_STATE magazine_launch_touch_sensors_state;
        MAGAZINE_TOUCH_SENSORS_STATE magazine_collect_touch_sensors_state;

        magazine_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.TS_ERROR;
        if (magazineLaunchTouchSensor.isPressed()){
            magazine_touch_sensors_state =  MAGAZINE_TOUCH_SENSORS_STATE.LAUNCH_TS_PRESSED;
            magazine_launch_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.LAUNCH_TS_PRESSED;
        } else {
            magazine_launch_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.LAUNCH_TS_NOT_PRESSED;
        }

        if (magazineCollectTouchSensor.isPressed()){
            magazine_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_PRESSED;
            magazine_collect_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_PRESSED;
        } else {
            magazine_collect_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_NOT_PRESSED;
        }

        if ((magazine_launch_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.LAUNCH_TS_NOT_PRESSED &&
                magazine_collect_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_NOT_PRESSED) ||
                (magazine_launch_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.LAUNCH_TS_PRESSED &&
                        magazine_collect_touch_sensors_state == MAGAZINE_TOUCH_SENSORS_STATE.COLLECT_TS_PRESSED)){
            magazine_touch_sensors_state = MAGAZINE_TOUCH_SENSORS_STATE.TS_ERROR;
        }

        return magazine_touch_sensors_state;
    }

    /**
     * Move Magazine to Collect position
     */
    public void moveMagazineToCollect(){
        if (magazineCollectTouchSensor.isPressed()) {
            magazineServo.setPower(0.0);
            magazinePosition = MAGAZINE_POSITION.AT_COLLECT;
        } else {
            magazineServo.setPower(-0.5);
        }
    }

    /**
     * Move Magazine to Launch position
     */
    public void moveMagazineToLaunch() {
        if (magazineLaunchTouchSensor.isPressed()) {
            magazineServo.setPower(0.0);
            magazinePosition = MAGAZINE_POSITION.AT_LAUNCH;
        } else {
            magazineServo.setPower(0.8);
        }
    }


    /**
     * Return Magazine Position
     */
    public MAGAZINE_POSITION getMagazinePosition(){
        senseMagazinePosition();
        return magazinePosition;
    }

    /**
     * Shake Magazine to shake rings in
     */
    public void shakeMagazine(int timeInMilliseconds){
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime shaketimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        shaketimer.reset();
        while (shaketimer.time() < timeInMilliseconds) {
            timer.reset();
            while (timer.time() < 50) {
                magazineServo.setPower(0.2);
            }
            timer.reset();
            while (timer.time() < 50) {
                magazineServo.setPower(0.2);
            }
        }
    }

}
