package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class SSElevator {

    public DcMotorEx elevatorMotorLeft;
    public DcMotorEx elevatorMotorRight;
    public TouchSensor touchSensor;

    //Gobilda (enter motor details later
    //Encoder count : enter details to be done


    public enum ELEVATOR_POSITION {
        LEVEL_LOW,
        LEVEL_HIGH,
    }


    // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
    //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Encoder Resolution: 537.7 PPR at the Output Shaft
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/

    // Amjad The motor used is a 1150 rpm motor
    //    https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-6mm-d-shaft-1150-rpm-36mm-gearbox-3-3-5v-encoder/

    public static double ENCODER_VALUE = 145.1; // The motor being used is 1150 Rpm motor with step of 145.1

    public static int ELEVATOR_LEVEL_LOW_POSITION_COUNT = 0; // 2023-12-21 calibrated value
    public static int ELEVATOR_LEVEL_HIGH_POSITION_COUNT = 2900;  // 2023-12-21 calibrated value

    public static int ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT = 100;
    public static int ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = 100;
    public static int ELEVATOR_LEVELMAX_POSITION_COUNT = ELEVATOR_LEVEL_HIGH_POSITION_COUNT;

    public static double POWER_LEVEL_RUN = 0.6;

    public ELEVATOR_POSITION elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;

    public int elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT;
    public int leftGetTargetPosition;

    public boolean runElevatorToLevelState = false;
    public double motorPowerToRun = POWER_LEVEL_RUN;

    private final LinearOpMode opMode;

    public SSElevator(HardwareMap hardwareMap, LinearOpMode opMode) {
        elevatorMotorLeft = hardwareMap.get(DcMotorEx.class, "elevator_motor_left");
        elevatorMotorRight = hardwareMap.get(DcMotorEx.class, "elevator_motor_right");
        touchSensor = hardwareMap.get(TouchSensor.class, "elevator_sensor");
        this.opMode = opMode;

        initElevator();
    }

    /**
     * Initialization for the Elevator
     */
    public void initElevator() {
        elevatorMotorLeft.setPositionPIDFCoefficients(5.0);
        elevatorMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        elevatorMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        elevatorMotorRight.setPositionPIDFCoefficients(5.0);
        elevatorMotorRight.setDirection(DcMotorEx.Direction.FORWARD);
        elevatorMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        resetElevator();
        initializeElevatorToLowPosition();

        if (getElevatorPosition() == ELEVATOR_POSITION.LEVEL_LOW) {
            turnElevatorBrakeModeOff();
        } else if (getElevatorPosition() == ELEVATOR_POSITION.LEVEL_HIGH) {
            turnElevatorBrakeModeOn();
        }
    }


    /**
     * Reset Elevator Encoder
     */
    public void resetElevator() {
        elevatorMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Method to set Elevator brake mode to ON when Zero (0.0) power is applied. <BR>
     * To be used when arm is above groundlevel
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnElevatorBrakeModeOn() {
        elevatorMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elevatorMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Method to set Elevator brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnElevatorBrakeModeOff() {
        elevatorMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        elevatorMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Method to run motor to set to the set position
     */
    public void runElevatorToLevel(double power) {
//        if (runElevatorToLevelState) {
//            elevatorMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            elevatorMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            elevatorMotorLeft.setPower(power);
//            elevatorMotorRight.setPower(power);
//            runElevatorToLevelState = false;
//        } else {
//            elevatorMotorRight.setPower(0.0);
//            elevatorMotorLeft.setPower(0.0);
//        }
    }

    /**
     * Initialize elevator to low position
     */
    public void initializeElevatorToLowPosition() {
        turnElevatorBrakeModeOff();
        elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT;
        elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
        elevatorMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;
    }

    /**
     * Move Elevator to levelLow Position
     */
    public void moveElevatorLevelLow() {
        turnElevatorBrakeModeOff();

        elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT;
        elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
        elevatorMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;

        //check if elevator sensor is on or off
        //otherwise create while loop and move elevator slightly down until sensor tells it is all the way down
//        while (!this.opMode.isStopRequested() && !isElevatorInLowPosition() ){
//            elevatorMotorLeft.setPower(POWER_LEVEL_RUN);
//            elevatorMotorRight.setPower(POWER_LEVEL_RUN);
//        }
//        elevatorMotorLeft.setPower(0.0);
//        elevatorMotorRight.setPower(0.0);
//        resetElevator();
//        turnElevatorBrakeModeOff();
        elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;
    }

    public void bringElevatorAllTheWayDown()
    {
        if ( isElevatorInLowPosition())
            return;

        turnElevatorBrakeModeOff();

        //check if elevator sensor is on or off
        //otherwise create while loop and move elevator slightly down until sensor tells it is all the way down
        while (!this.opMode.isStopRequested() && !isElevatorInLowPosition() ){
            elevatorPositionCount = elevatorPositionCount - ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT;
            elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
            elevatorMotorRight.setTargetPosition(elevatorPositionCount);
            motorPowerToRun = POWER_LEVEL_RUN;
            runElevatorToLevelState = true;

            runElevatorToLevel(motorPowerToRun);
        }
        elevatorMotorLeft.setPower(0.0);
        elevatorMotorRight.setPower(0.0);
        resetElevator();
        elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;

//        turnElevatorBrakeModeOff();
    }
    /**
     * Move Elevator to levelHigh Position
     */
    public void moveElevatorLevelHigh() {
        turnElevatorBrakeModeOn();

        elevatorPositionCount = ELEVATOR_LEVEL_HIGH_POSITION_COUNT;
        elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
        elevatorMotorRight.setTargetPosition(elevatorPositionCount);
        leftGetTargetPosition = elevatorMotorRight.getTargetPosition();
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.LEVEL_HIGH;
    }

    /**
     * Move Elevator Slightly Down
     */
    public void moveSSElevatorSlightlyDown() {
        turnElevatorBrakeModeOn();
        elevatorPositionCount = elevatorPositionCount - ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT;
        elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
        elevatorMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;

        resetElevator();
        elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT;
    }

    /**
     * Move Elevator Slightly Up
     */
    public void moveSSElevatorSlightlyUp() {
        turnElevatorBrakeModeOn();
        elevatorPositionCount = elevatorPositionCount + ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT;
        elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
        elevatorMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;
    }

    /**
     * Return elevator state
     */
    public ELEVATOR_POSITION getElevatorPosition() {
        return elevatorPosition;
    }

    /**
     * Return elevator current position
     */
    public int currentLeftEncoderValue() {
        return elevatorMotorLeft.getCurrentPosition();
    }

    public int currentRightEncoderValue() {
        return elevatorMotorRight.getCurrentPosition();
    }

    public boolean isElevatorInLowPosition() {
        return touchSensor.isPressed();
    }
}


