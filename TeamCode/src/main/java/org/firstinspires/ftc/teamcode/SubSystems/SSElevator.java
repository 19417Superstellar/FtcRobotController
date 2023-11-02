package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SSElevator {
    public DcMotorEx elevatorMotorLeft;
    public DcMotorEx elevatorMotorRight;
    public TouchSensor touchSensor;



    public enum ELEVATOR_POSITION {
        LEVEL_LOW,
        LEVEL_MID,
        LEVEL_HIGH
    }



    public static double ENCODER_VALUE = 145.1; // The motor being used is 1150 Rpm motor with step of 145.1

    public static int ELEVATOR_LEVEL_LOW_POSITION_COUNT = 0; // 2023-12-21 calibrated value
    public static int ELEVATOR_LEVEL_MID_POSITION_COUNT = 1450;
    public static int ELEVATOR_LEVEL_HIGH_POSITION_COUNT = 2900;  // 2023-12-21 calibrated value

    public static int ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT = 300;
    public static int ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = 300;
    public static int ELEVATOR_LEVELMAX_POSITION_COUNT = ELEVATOR_LEVEL_HIGH_POSITION_COUNT;

    public static double POWER_LEVEL_RUN = .9;

    public ELEVATOR_POSITION elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;

    public int elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT;
    public int leftGetTargetPosition;

    public boolean runElevatorToLevelState = false;
    public double motorPowerToRun = POWER_LEVEL_RUN;

    public boolean elevatorNeedsToGoDown = false;

    public Telemetry telemetry;
    public SSElevator(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        elevatorMotorLeft = hardwareMap.get(DcMotorEx.class, "elevator_motor_left");
        elevatorMotorRight = hardwareMap.get(DcMotorEx.class, "elevator_motor_right");
        touchSensor = hardwareMap.get(TouchSensor.class, "elevator_sensor");

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

    public void turnElevatorBrakeModeOn() {
        elevatorMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elevatorMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void turnElevatorBrakeModeOff() {
        elevatorMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        elevatorMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void runElevatorToLevel(double power) {
        if (elevatorNeedsToGoDown ) {
            if (!isElevatorInLowPosition()) {
                elevatorPositionCount = elevatorPositionCount - ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT;
                elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
                elevatorMotorRight.setTargetPosition(elevatorPositionCount);
                motorPowerToRun = POWER_LEVEL_RUN;
                runMotors(motorPowerToRun);
            }
            else
            {
                elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;
                elevatorNeedsToGoDown = false;
                resetElevator();
                stopMotors();
            }
            return;
        }

        if (runElevatorToLevelState) {
            runMotors(power);
            runElevatorToLevelState = false;
        } else {
            stopMotors();
        }
    }

    private void runMotors(double power) {
        elevatorMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorMotorLeft.setPower(power);
        elevatorMotorRight.setPower(power);
    }

    private void stopMotors() {
        elevatorMotorRight.setPower(0.0);
        elevatorMotorLeft.setPower(0.0);
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

    public void bringElevatorAllTheWayDown()
    {
        if ( isElevatorInLowPosition()) {
            elevatorNeedsToGoDown = false;
        }
        else
        {
            elevatorNeedsToGoDown = true;
        }


    }
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

    public void moveElevatorLevelMid() {
        turnElevatorBrakeModeOn();

        elevatorPositionCount = ELEVATOR_LEVEL_MID_POSITION_COUNT;
        elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
        elevatorMotorRight.setTargetPosition(elevatorPositionCount);
        leftGetTargetPosition = elevatorMotorRight.getTargetPosition();
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.LEVEL_MID;
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

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("=============");
    }
}