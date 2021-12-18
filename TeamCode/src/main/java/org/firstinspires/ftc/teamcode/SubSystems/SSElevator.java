package org.firstinspires.ftc.teamcode.SubSystems;
/*
 *THE Elevator subsystem class
*/

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SSElevator {

    public DcMotorEx elevatorMotor;
    //Gobilda (enter motor details later
    //Encoder count : enter details to be done


    public enum ELEVATOR_POSITION {
        LEVEL_0,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3
    }


    // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
    //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Encoder Resolution: 537.7 PPR at the Output Shaft
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static double ENCODER_VALUE = 537.7;
    public static int baselineEncoderCount = 0;
    public static int ELEVATOR_LEVEL0_POSITION_COUNT = 0;
    public static int ELEVATOR_LEVEL1_POSITION_COUNT = 750;//  TODO : Determine by experimentation
    public static int ELEVATOR_LEVEL2_POSITION_COUNT = 1200;//  TODO : Determine by experimentation
    public static int ELEVATOR_LEVEL3_POSITION_COUNT = 1900;//  TODO : Determine by experimentation
    //MAX 2200
    public static int ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT = 100;
    public static int ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = 100;
    public static int ELEVATOR_LEVELMAX_POSITION_COUNT = 2200;

    public static double POWER_NO_CARGO = 0.5;
    public static double POWER_WITH_CARGO = 0.5;

    public ELEVATOR_POSITION elevatorPosition = ELEVATOR_POSITION.LEVEL_0;
    public ELEVATOR_POSITION previousPosition = ELEVATOR_POSITION.LEVEL_0;

    public int elevatorPositionCount = ELEVATOR_LEVEL0_POSITION_COUNT;



    public SSElevator(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotorEx.class,"elevator_motor");
        initElevator();
    }

    /**
     * Initialization for the Elevator
     */
    public void initElevator(){
        elevatorMotor.setPositionPIDFCoefficients(5.0);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //resetElevator();
        moveElevatorLevel0();
        turnElevatorBrakeModeOn();
    }

    /**
     * Reset Elevator Encoder
     */
    public void resetElevator(){
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Method to set Elevator brake mode to ON when Zero (0.0) power is applied. <BR>
     * To be used when arm is above groundlevel
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnElevatorBrakeModeOn(){
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Method to set Elevator brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnElevatorBrakeModeOff(){
        elevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public boolean runElevatorToLevelState = false;
    public double motorPowerToRun = POWER_NO_CARGO;

    /**
     * Method to run motor to set to the set position
     */
    public void runElevatorToLevel(double power){
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    /**
     * Move Elevator to level12 Position
     */
    public void moveElevatorLevel2() {
        turnElevatorBrakeModeOn();
        elevatorPositionCount = ELEVATOR_LEVEL2_POSITION_COUNT + baselineEncoderCount;
        elevatorMotor.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_WITH_CARGO;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.LEVEL_2;
    }

    /**
     * Move Elevator to level3 Position
     */
    public void moveElevatorLevel3() {
        turnElevatorBrakeModeOn();

        elevatorPositionCount = ELEVATOR_LEVEL3_POSITION_COUNT + baselineEncoderCount;
        elevatorMotor.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_WITH_CARGO;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.LEVEL_3;
    }

    /**
     * Move Elevator Slightly Down
     */
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

    /**
     * Move Elevator Slightly Up
     */
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

    /**
     * Return elevator state
     */
    public ELEVATOR_POSITION getElevatorPosition() {

        return elevatorPosition;
    }

    /**
     * Return elevator current position
     */
    public int currentEncoderValue(){
        return elevatorMotor.getCurrentPosition();
    }

}
