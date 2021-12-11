package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SSArm {


    public DcMotorEx armMotor;
    public Servo gripServo;
    //Gobilda (enter motor details later
    //Encoder count : enter details to be done


    public enum ARM_POSITION {
        ARM_PICKUP,
        ARM_DROP,
        ARM_PARKED,
        ARM_DROPDOWN;
    }

    public enum GRIP_SERVO_STATE {
        GRIP_OPEN,
        GRIP_CLOSE;
    }

    public GRIP_SERVO_STATE gripServoState = GRIP_SERVO_STATE.GRIP_OPEN;

    // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
    //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Encoder Resolution: 537.7 PPR at the Output Shaft
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static double ENCODER_VALUE = 537.7;
    public static int baselineEncoderCount = 0;
    public static int ARM_BASELINE_POSITION_COUNT = 0;
    public static int ARM_PICKUP_POSITION_COUNT = -700;//  TODO : Determine by experimentation
    public static int ARM_CAPSTONE_POSITION_COUNT = -450;//  TODO : Determine by experimentation
    public static int ARM_PARKED_POSITION_COUNT = 0;//  TODO : Determine by experimentation
    //MAX 2200


    public static double POWER_ARM_UP = 0.5;
    public static double POWER_CAPSTONE_UP = 0.5;
    public static double POWER_PARK_UP = 0;
    public static double GRIP_OPEN_POSITION = 1.0;
    public static double GRIP_CLOSE_POSITION = 0;


    public ARM_POSITION armPosition = ARM_POSITION.ARM_PARKED;
    public ARM_POSITION previousPosition = ARM_POSITION.ARM_PARKED;


    public int armPositionCount = ARM_PICKUP_POSITION_COUNT;



    public SSArm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class,"arm_motor");
        gripServo = hardwareMap.get(Servo.class,"grip_motor");

    }

    /**
     * Initialization for the Arm
     */
    public void initArm(){
        armMotor.setPositionPIDFCoefficients(5.0); //  TODO : Determine by experimentation
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);  //TODO : Determine by experimentation
        //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //resetArm();
        moveArmPickup();
        turnArmBrakeModeOn();
    }

    /**
     * Reset Elevator Encoder
     */
    public void resetArm(){
        //DcMotor.RunMode runMode = elevatorMotor.getMode();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setMode(runMode);
    }

    /**
     * Method to set Elevator brake mode to ON when Zero (0.0) power is applied. <BR>
     * To be used when arm is above groundlevel
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOn(){
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Method to set Elevator brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOff(){
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public boolean runArmToLevelState = false;
    public double motorPowerToRun = POWER_ARM_UP;

    /**
     * Method to run motor to set to the set position
     */
    public void runArmToLevel(double power){
        //elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (runArmToLevelState || armMotor.isBusy() ){
            armMotor.setPower(power);
            runArmToLevelState = false;
        } else {
            armMotor.setPower(0.0);
        }
    }


    /**
     * Move Elevator to level0 Position
     */
    public void moveArmPickup() {
        turnArmBrakeModeOff();

        armPositionCount = ARM_PICKUP_POSITION_COUNT + baselineEncoderCount;
        armMotor.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_PICKUP;
    }

    /**
     * Move arm to Capstone Position
     */

    public void moveArmDrop() {
        turnArmBrakeModeOn();

        armPositionCount = ARM_CAPSTONE_POSITION_COUNT + baselineEncoderCount;
        armMotor.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_DROP;
    }

    public void moveArmParked() {
        turnArmBrakeModeOn();

        armPositionCount = ARM_PARKED_POSITION_COUNT + baselineEncoderCount;
        armMotor.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_PARKED;
    }

    public void dropBelowCapstone() {
        turnArmBrakeModeOn();

        armPositionCount = ARM_CAPSTONE_POSITION_COUNT + baselineEncoderCount - 50;
        armMotor.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_DROPDOWN;
    }

    public void setGripOpen() {
       if (gripServoState != GRIP_SERVO_STATE.GRIP_OPEN) {
           gripServo.setPosition(GRIP_OPEN_POSITION);
           gripServoState = GRIP_SERVO_STATE.GRIP_OPEN;
       }
    }

    public void setGripClose() {
        if (gripServoState != GRIP_SERVO_STATE.GRIP_CLOSE) {
            gripServo.setPosition(GRIP_CLOSE_POSITION);
            gripServoState = GRIP_SERVO_STATE.GRIP_CLOSE;
        }
    }



    //function to determinie POWER_WITH_CARGO or POWER_WITH_NO_CARGO

    public ARM_POSITION getArmPosition() {

        return armPosition;
    }

    public GRIP_SERVO_STATE getGripServoState() {
        return gripServoState;
    }

    public int currentEncoderValue(){

        return armMotor.getCurrentPosition();
    }

}

