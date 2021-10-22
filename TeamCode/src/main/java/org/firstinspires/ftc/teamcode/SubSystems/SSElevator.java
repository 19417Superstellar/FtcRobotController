package org.firstinspires.ftc.teamcode.SubSystems;
/**
 *
*/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SSElevator {

    public DcMotorEx SSElevatorMotor;
    //Gobilda (enter motor details later
    //Encoder count : enter details to be done


    public enum ELEVATOR_POSITION {
        LEVEL_0,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3
    }


    // Encoder values for xxx Gobilda xxxx motor yyyy encoder counts / revolution
    public static int baselineEncoderCount = 0;
    public static int ELEVATOR_PARKED_POSITION_COUNT = 0;
    public static int ELEVATOR_LEVEL0_POSITION_COUNT = 178;//-250;
    public static int ELEVATOR_LEVEL1_POSITION_COUNT = 357;//-500 ;
    public static int ELEVATOR_LEVEL2_POSITION_COUNT = 500;//-700;
    public static int ELEVATOR_LEVEL3_POSITION_COUNT = 525;

    public static double POWER_NO_CARGO = 0.5;
    public static double POWER_WITH_CARGO = 0.3;

    public ELEVATOR_POSITION currentPosition = ELEVATOR_POSITION.LEVEL_0;
    public ELEVATOR_POSITION previousPosition = ELEVATOR_POSITION.LEVEL_0;



    public SSElevator(HardwareMap hardwareMap) {
       DcMotorEx SSElevatorMotor = hardwareMap.get(DcMotorEx.class,"elevator_motor");

    }

    LinearOpMode opModepassed;

    /**
     * Initialization for the Elevator
     */
    public void initElevator(){

        SSElevatorMotor.setPositionPIDFCoefficients(5.0);
        SSElevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        SSElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetElevator();
        moveElevatorLevel0();
        turnElevatorBrakeModeOn();
    }

    /**
     * Reset Elevator Encoder
     */
    public void resetElevator(){
        DcMotor.RunMode runMode = SSElevatorMotor.getMode();
        SSElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SSElevatorMotor.setMode(runMode);
    }

    /**
     * Method to set Arm brake mode to ON when Zero (0.0) power is applied. <BR>
     * To be used when arm is above groundlevel
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnElevatorBrakeModeOn(){
        SSElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Method to set Arm brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnElevatorBrakeModeOff(){

        SSElevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
    
    public boolean runElevatorToLevelState = false;
    public double motorPowerToRun = POWER_NO_CARGO;

    /**
     * Method to run motor to set to the set position
     */
    public void runElevatorToLevel(double power){
        SSElevatorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (runElevatorToLevelState == true || SSElevatorMotor.isBusy() == true){
            SSElevatorMotor.setPower(power);
            runElevatorToLevelState = false;
        } else {
            SSElevatorMotor.setPower(0.0);
        }
    }

    /**
     * Move Elevator to level0 Position
     */
    public void moveElevatorLevel0() {
        turnElevatorBrakeModeOff();
        SSElevatorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        SSElevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SSElevatorMotor.setTargetPosition(ELEVATOR_LEVEL0_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_NO_CARGO;
        runElevatorToLevelState = true;
        currentPosition = ELEVATOR_POSITION.LEVEL_0;
    }

    /**
     * Move Elevator to level1 Position
     */

    public void moveElevatorLevel1() {
        turnElevatorBrakeModeOff();
        SSElevatorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        SSElevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SSElevatorMotor.setTargetPosition(ELEVATOR_LEVEL1_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_WITH_CARGO;
        runElevatorToLevelState = true;
        currentPosition = ELEVATOR_POSITION.LEVEL_1;
    }

    public void moveElevatorLevel2() {
        turnElevatorBrakeModeOff();
        SSElevatorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        SSElevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SSElevatorMotor.setTargetPosition(ELEVATOR_LEVEL2_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_WITH_CARGO;
        runElevatorToLevelState = true;
        currentPosition = ELEVATOR_POSITION.LEVEL_2;
    }

    public void moveElevatorLevel3() {
        turnElevatorBrakeModeOff();
        SSElevatorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        SSElevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SSElevatorMotor.setTargetPosition(ELEVATOR_LEVEL3_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_WITH_CARGO;
        runElevatorToLevelState = true;
        currentPosition = ELEVATOR_POSITION.LEVEL_3;
    }


    }
