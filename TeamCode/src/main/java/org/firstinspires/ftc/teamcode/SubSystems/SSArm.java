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
        ARM_PARKED;
    }

    public enum GRIP_STATE {
        GRIP_OPEN,
        GRIP_CLOSE;
    }


    // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
    //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Encoder Resolution: 537.7 PPR at the Output Shaft
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static double ENCODER_VALUE = 537.7;
    public static int baselineEncoderCount = 0;
    public static int ARM_BASELINE_POSITION_COUNT = 0;
    public static int ARM_PICKUP_POSITION_COUNT = 0;//  TODO : Determine by experimentation
    public static int ARM_CAPSTONE_POSITION_COUNT = 2000;//  TODO : Determine by experimentation
    public static int ARM_PARK_POSITION_COUNT = 2500;//  TODO : Determine by experimentation
    //MAX 2200


    public static double POWER_ARM_UP = 0;
    public static double POWER_CAPSTONE_UP = 0;
    public static double POWER_PARK_UP = 0;


    public ARM_POSITION armPosition = ARM_POSITION.ARM_PARKED;
    public ARM_POSITION previousPosition = ARM_POSITION.ARM_PARKED;


    public int armPositionCount = ARM_PICKUP_POSITION_COUNT;



    public SSArm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class,"arm_motor");

    }

    public SSArm(HardwareMap hardwareMap) {
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
    public void resetElevator(){
        //DcMotor.RunMode runMode = elevatorMotor.getMode();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setMode(runMode);
    }

    /**
     * Method to set Elevator brake mode to ON when Zero (0.0) power is applied. <BR>
     * To be used when arm is above groundlevel
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnElevatorBrakeModeOn(){
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Method to set Elevator brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnElevatorBrakeModeOff(){
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public boolean runElevatorToLevelState = false;
    public double motorPowerToRun = POWER_NO_CARGO;

    /**
     * Method to run motor to set to the set position
     */
    public void runElevatorToLevel(double power){
        //elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (runElevatorToLevelState || elevatorMotor.isBusy() ){
            elevatorMotor.setPower(power);
            runElevatorToLevelState = false;
        } else {
            elevatorMotor.setPower(0.0);
        }
    }


    /**
     * Move Elevator to level0 Position
     */
    public void moveElevatorLevel0() {
        turnElevatorBrakeModeOff();

        elevatorPositionCount = ELEVATOR_LEVEL0_POSITION_COUNT + baselineEncoderCount;
        elevatorMotor.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_NO_CARGO;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.LEVEL_0;
    }

    /**
     * Move Elevator to level1 Position
     */

    public void moveElevatorLevel1() {
        turnElevatorBrakeModeOn();

        elevatorPositionCount = ELEVATOR_LEVEL1_POSITION_COUNT + baselineEncoderCount;
        elevatorMotor.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_WITH_CARGO;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.LEVEL_1;
    }

    public void moveElevatorLevel2() {
        turnElevatorBrakeModeOn();

        elevatorPositionCount = ELEVATOR_LEVEL2_POSITION_COUNT + baselineEncoderCount;
        elevatorMotor.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_WITH_CARGO;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.LEVEL_2;
    }

    public void moveElevatorLevel3() {
        turnElevatorBrakeModeOn();

        elevatorPositionCount = ELEVATOR_LEVEL3_POSITION_COUNT + baselineEncoderCount;
        elevatorMotor.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_WITH_CARGO;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.LEVEL_3;
    }
    public void moveSSElevatorSlightlyDown() {
        if ((elevatorPositionCount > ELEVATOR_LEVEL0_POSITION_COUNT) &&
                elevatorPositionCount <= ELEVATOR_LEVELMAX_POSITION_COUNT - ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT) {
            turnElevatorBrakeModeOn();
            elevatorPositionCount = elevatorPositionCount + ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT;
            elevatorMotor.setTargetPosition(elevatorPositionCount);
            motorPowerToRun = POWER_NO_CARGO;
            runElevatorToLevelState = true;
        }
    }

    public void moveSSElevatorSlightlyUp(){
        if ((elevatorPositionCount > ELEVATOR_LEVEL0_POSITION_COUNT) &&
                elevatorPositionCount <= ELEVATOR_LEVELMAX_POSITION_COUNT - ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT){
            turnElevatorBrakeModeOn();
            elevatorPositionCount = elevatorPositionCount + ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT;
            elevatorMotor.setTargetPosition(elevatorPositionCount);
            motorPowerToRun = POWER_WITH_CARGO;
            runElevatorToLevelState = true;
        }
    }

    //function to determinie POWER_WITH_CARGO or POWER_WITH_NO_CARGO

    public ELEVATOR_POSITION getElevatorPosition() {

        return elevatorPosition;
    }

    public int currentEncoderValue(){

        return elevatorMotor.getCurrentPosition();
    }

}

