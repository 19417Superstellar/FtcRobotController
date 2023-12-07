package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SSElevator {
    public DcMotorEx elevatorMotorLeft;
    public DcMotorEx elevatorMotorRight;

    public VoltageSensor voltageSensor;

    public double voltageThreshold = 9.0; // 9 volts

    public enum ELEVATOR_POSITION {
        LEVEL_LOW,
        LEVEL_MID,
        LEVEL_HIGH,
        LEVEL_INTAKE
    }

    public static int ELEVATOR_LEVEL_INTAKE_POSITION_COUNT = 0;
    public static int ELEVATOR_LEVEL_LOW_POSITION_COUNT = 450;
    public static int ELEVATOR_LEVEL_MID_POSITION_COUNT = 700;
    public static int ELEVATOR_LEVEL_HIGH_POSITION_COUNT = 1000;
    public static int ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT = 150      ;
    public static int ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = 150;

    public static double POWER_LEVEL_RUN = .8;
    public ELEVATOR_POSITION elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;

    public int elevatorPositionCount = ELEVATOR_LEVEL_INTAKE_POSITION_COUNT;
    public boolean runElevatorToLevelState = false;
    public double motorPowerToRun = POWER_LEVEL_RUN;

    public boolean elevatorNeedsToGoDown = false;

    public Telemetry telemetry;
    public SSElevator(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        elevatorMotorLeft = hardwareMap.get(DcMotorEx.class, "elevator_motor_left");
        elevatorMotorRight = hardwareMap.get(DcMotorEx.class, "elevator_motor_right");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
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
        elevatorMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
        elevatorMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        moveElevatorPickPosition();
        resetElevator();
        if (getElevatorPosition() == ELEVATOR_POSITION.LEVEL_LOW) {
            turnElevatorBrakeModeOff();
        } else {//if (getElevatorPosition() == ELEVATOR_POSITION.LEVEL_HIGH) {
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
            if (!isMotorStalled()) {
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
    public void moveElevatorPickPosition() {
        turnElevatorBrakeModeOff();
        elevatorPositionCount = ELEVATOR_LEVEL_INTAKE_POSITION_COUNT;
        elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
        elevatorMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.LEVEL_INTAKE;
    }
    public void bringElevatorAllTheWayDown() {
        if ( isMotorStalled()) {
            elevatorNeedsToGoDown = false;
        }
        else {
            elevatorNeedsToGoDown = true;
        }
    }
    public void moveElevatorLevelHigh() {
        turnElevatorBrakeModeOn();

        elevatorPositionCount = ELEVATOR_LEVEL_HIGH_POSITION_COUNT;
        elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
        elevatorMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.LEVEL_HIGH;
    }

    public void moveElevatorLevelLow() {
        turnElevatorBrakeModeOn();

        elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT;
        elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
        elevatorMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.LEVEL_LOW;
    }

    public void moveElevatorLevelMid() {
        turnElevatorBrakeModeOn();

        elevatorPositionCount = ELEVATOR_LEVEL_MID_POSITION_COUNT;
        elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
        elevatorMotorRight.setTargetPosition(elevatorPositionCount);
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
        if ( elevatorPositionCount < 0 ) {
            elevatorPositionCount= 0;
        }
      elevatorMotorLeft.setTargetPosition(elevatorPositionCount);
        elevatorMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
       runElevatorToLevelState = true;

        //resetElevator();
        //elevatorPositionCount = ELEVATOR_LEVEL_LOW_POSITION_COUNT;
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
       // runElevatorToLevelState = true;
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

    public boolean isMotorStalled() {
        return voltageSensor.getVoltage() <= this.voltageThreshold;
    }

    public void printDebugMessages(){
        //******  debug ******
        telemetry.addData("elevator_motor_encoder_left", currentLeftEncoderValue());
        telemetry.addData("elevator_motor_encoder_right",currentRightEncoderValue());
        telemetry.addData("elevator_motor_encoder val ", elevatorPositionCount);
        telemetry.addData("elevator_motor_stalled",this.isMotorStalled());
    }
}