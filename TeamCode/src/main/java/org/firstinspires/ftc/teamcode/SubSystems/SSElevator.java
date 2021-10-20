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
     * Initialization for the Arm
     */
    public void initArm(){

        SSElevatorMotor.setPositionPIDFCoefficients(5.0);
        SSElevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        SSElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetElevator();
        moveElevatorParkedPosition();
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
    public void turnArmBrakeModeOn(){
        SSElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Method to set Arm brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOff(){
        SSElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    public boolean runElevatorToLevelState = false;
    public double motorPowerToRun = POWER_NO_CARGO;

    /**
     * Method to run motor to set to the set position
     */
    public void runElevatorToLevel(double power){
        SSElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (runElevatorToLevelState == true || SSElevatorMotor.isBusy() == true){
            SSElevatorMotor.setPower(power);
            runElevatorToLevelState = false;
        } else {
            SSElevatorMotor.setPower(0.0);
        }
    }

    /**
     * Move Arm to Park Position
     */
    public void moveElevatorLevel0() {
        turnArmBrakeModeOff();
        SSElevatorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        SSElevator setDirectionReverse;
        SSElevatorMotor.setTargetPosition(ELEVATOR_LEVEL0_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_WITH_CARGO;
        runElevatorToLevelState = true;
        currentPosition = ELEVATOR_POSITION.LEVEL_0;
    }

    /**
     * Move Arm to Holding up position of Wobble and Ring in TeleOp Mode
     */
    public void moveElevatorHoldUpCargo() {
        turnArmBrakeModeOn();
        SSElevatorMotor.setTargetPosition(ELEVATOR_HOLD_UP_CARGO_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_WITH_CARGO;
        runElevatorToLevelState = true;
        currentElevatorPosition = ELEVATOR_POSITION.HOLD_UP_CARGO;
    }


    /**
     * Move Arm to Drop Wobble goal and Ring in TeleOp Mode
     */
    public void moveArmDropWobbleRingPosition() {
        turnArmBrakeModeOn();
        SSElevatorMotor.setTargetPosition(ARM_DROP_WOBBLE_RING_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_WITH_CARGO;
        runElevatorToLevelState = true;
        currentArmPosition = ARM_POSITION.DROP_WOBBLE_RING;
    }

    /**
     * Move Arm to Drop Wobble Goal in autonomous
     */
    public void moveArmDropWobbleAutonomousPosition() {
        turnArmBrakeModeOn();
        SSElevatorMotor.setTargetPosition(ARM_DROP_WOBBLE_AUTONOMOUS_POSITION + baselineEncoderCount);
        motorPowerToRun = POWER_WITH_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.DROP_WOBBLE_AUTONOMOUS;
    }


    /**
     * Move arm to Pick Wobble Position
     */
    public void moveArmPickWobblePosition() {
        turnArmBrakeModeOn();
        SSElevatorMotor.setTargetPosition(ARM_PICK_WOBBLE_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_NO_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.PICK_WOBBLE;
    }

    /**
     * Move Arm to Pick Ring Position
     */
    public void moveArmPickRingPosition() {
        turnArmBrakeModeOn();
        SSElevatorMotor.setTargetPosition(ARM_PICK_RING_POSITION_COUNT + baselineEncoderCount);
        motorPowerToRun = POWER_NO_WOBBLEGOAL;
        runArmToLevelState = true;
        currentArmPosition = ARM_POSITION.PICK_RING;
    }

    /**
     * Method for moving arm based on trigger inputs.
     * Depending on the grip condition in the pick wobble goal position or the pic ring position
     * the next motion is determined
     */
    public void moveArmByTrigger() {
        if ((currentArmPosition== ARM_POSITION.PARKED)) {
            previousArmPosition = currentArmPosition;
            moveArmPickWobblePosition();
            openGrip();
            return;
        }

        //To go to ring position, grip should be open and triggered again from pick position
        if ((currentArmPosition== ARM_POSITION.PICK_WOBBLE)  &&
                (getGripServoState() == GRIP_SERVO_STATE.OPENED)) {
            previousArmPosition = currentArmPosition;
            moveArmPickRingPosition();
            return;
        }

        if ((currentArmPosition== ARM_POSITION.PICK_RING)  &&
                (getGripServoState() == GRIP_SERVO_STATE.OPENED)) {
            previousArmPosition = currentArmPosition;
            closeGrip();
            moveArmParkedPosition();
            return;
        }

        // After closing grip in pick wobble or ring position - assumes holding of wobble goal is done
        if ((currentArmPosition== ARM_POSITION.PICK_WOBBLE)  &&
                (previousArmPosition == ARM_POSITION.PARKED) &&
                (getGripServoState() == GRIP_SERVO_STATE.CLOSED)) {
            previousArmPosition = currentArmPosition;
            moveArmHoldUpWobbleRingPosition();
            return;
        }

        // After closing grip in pick wobble or ring position - assumes holding of wobble goal is done
        if ((currentArmPosition== ARM_POSITION.PICK_RING)  &&
                (getGripServoState() == GRIP_SERVO_STATE.CLOSED)) {
            previousArmPosition = currentArmPosition;
            moveArmHoldUpWobbleRingPosition();
            return;
        }

        if ((currentArmPosition== ARM_POSITION.HOLD_UP_WOBBLE_RING)  &&
                (getGripServoState() == GRIP_SERVO_STATE.CLOSED)) {
            previousArmPosition = currentArmPosition;
            moveArmDropWobbleRingPosition();
            return;
        }

        if ((currentArmPosition== ARM_POSITION.DROP_WOBBLE_RING)  &&
                (getGripServoState() == GRIP_SERVO_STATE.OPENED)) {
            previousArmPosition = currentArmPosition;
            closeGrip();
            moveArmParkedPosition();
            return;
        }

        if ((currentArmPosition== ARM_POSITION.DROP_WOBBLE_RING)  &&
                (getGripServoState() == GRIP_SERVO_STATE.CLOSED)) {
            previousArmPosition = currentArmPosition;
            moveArmHoldUpWobbleRingPosition();
            return;
        }

    }



    /**



    /**
     * Return the current Arm Position
     */
    public ARM_POSITION getCurrentArmPosition() {
        return currentArmPosition;
    }

    /**
