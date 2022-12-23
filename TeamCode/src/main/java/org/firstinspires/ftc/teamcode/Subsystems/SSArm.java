package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SSArm {


    public DcMotorEx armMotorRight;
    public DcMotorEx armMotorLeft;
    //add 2 motors left and right
    //Gobilda (enter motor details later
    //Encoder count : enter details to be done


    public enum ARM_POSITION {
        ARM_POSITION_INTAKE_FORWARD,
        ARM_POSITION_LOW,
        ARM_POSITION_MID,
        ARM_POSITION_HIGH,
        ARM_POSITION_INTAKE_REAR,
        ARM_POSITION_RANDOM
    }


    // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
    //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Encoder Resolution: 537.7 PPR at the Output Shaft
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    //public static double ENCODER_VALUE = 537.7;
    // public static int ARM_BASELINE_POSITION_COUNT = 0;
    //Use testarm teleop to verify if armMotorLeft and armMotorRight use the same encoder values
    //if not, add different constants for the two motors
    //public static int ARM_CUP_POSITION_COUNT = 375;//  Determine by experimentation
    public static int ARM_FORWARD_INTAKE_POSITION_COUNT = 0; //Determine by experimentation
    public static int ARM_LOW_POSITION_COUNT = 240; // Determine by experimentation
    public static int ARM_MID_POSITION_COUNT = 350; // Determine by experimentation

    // Preventing the arm from going back till we mechanically reinforce it so it has enough
    // power to come back forward
    public static int ARM_HIGH_POSITION_COUNT = 400; // Determine by experimentation
    public static int ARM_REAR_INTAKE_POSITION_COUNT = 400; // Determine by experimentation

    public static int ARM_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = 50; // Determine by experimentation
    public static int ARM_DELTA_SLIGHTLY_UP_DELTA_COUNT = 50; // Determine by experimentation
    //add count positions for different junctions
    //MAX 2200

    private static final double ARM_FORWARD_INTAKE_POSITION_ANGLE = 275;
    private static final double ARM_LOW_INTAKE_POSITION_ANGLE = 225;
    private static final double ARM_MID_INTAKE_POSITION_ANGLE = 135;
    private static final double ARM_HIGH_INTAKE_POSITION_ANGLE = 135;

    private static final double PID_CONSTANT = 3.5;
    public static double POWER_ARM_UP = 0.5;


    public ARM_POSITION armPosition = ARM_POSITION.ARM_POSITION_INTAKE_FORWARD;
    //public ARM_POSITION previousPosition = ARM_POSITION.ARM_POSITION_INTAKE_FORWARD;


    public int armPositionCount = ARM_FORWARD_INTAKE_POSITION_COUNT;


    public SSArm(HardwareMap hardwareMap) {
        armMotorLeft = hardwareMap.get(DcMotorEx.class, "arm_motor_left");
        armMotorRight = hardwareMap.get(DcMotorEx.class, "arm_motor_right");
        initArm();
    }

    /**
     * Initialization for the Arm
     */
    public void initArm() {
        resetArm();
        turnArmBrakeModeOff();
        // This is default PID value - it is then set for each level later
        armMotorLeft.setPositionPIDFCoefficients(3.5);
        armMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);

        armMotorRight.setPositionPIDFCoefficients(3.5);
        armMotorRight.setDirection(DcMotorEx.Direction.REVERSE);

        armPosition = ARM_POSITION.ARM_POSITION_INTAKE_FORWARD;
    }


    public void resetArm() {
        //DcMotorEx.RunMode runMode = elevatorMotor.getMode();
        armMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //armMotor.setMode(runMode);
    }

    /**
     * Gets a calculated P (PID) coefficient based on the angle the arm
     * is supposed to go to.
     * This is detailed at : https://www.ctrlaltftc.com/feedforward-control
     * The always return positive value even though cosine for some angles is negative
     * You can think of angles as 4 quadrants, each from 0 to 90 and repeating counter clockwise
     * So either you can provide an angle from 0-90 degree or 0-360 (CCwise)
     * @param angle : Target angle the arm will go to. 0 is 3 o'clock, 90 is 12 o'clock.
     *              increasing counter clock wise.
     * @return - Needed P value for PID control
     */
    private double getPIDValue(double angle) {
        double power = Math.cos(angle) * SSArm.PID_CONSTANT;
        return Math.abs(power);
    }

    /**
     * Method to set Elevator brake mode to ON when Zero (0.0) power is applied. <BR>
     * To be used when arm is above groundlevel
     * setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOn() {
        armMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    /**
     * Method to set Elevator brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOff() {
        armMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        armMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    }

    public boolean runArmToLevelState = false;
    public double motorPowerToRun = POWER_ARM_UP;

    /**
     * Method to run motor to set to the set position
     */
    public void runArmToLevel(double power) {
        armMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (runArmToLevelState ) {
            armMotorLeft.setPower(power);
            armMotorRight.setPower(power);
            runArmToLevelState = false;
            armMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            armMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        } else {
            armMotorLeft.setPower(0.0);
            armMotorRight.setPower(0.0);

        }
    }


    /**
     * Move Elevator to level0 Position
     */
    public void moveArmIntakeForward() {
        turnArmBrakeModeOn();
        armPositionCount = ARM_FORWARD_INTAKE_POSITION_COUNT;
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);

        double pidVal = getPIDValue(ARM_FORWARD_INTAKE_POSITION_ANGLE);
        armMotorLeft.setPositionPIDFCoefficients(getPIDValue(pidVal));
        armMotorRight.setPositionPIDFCoefficients(pidVal);

        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_INTAKE_FORWARD;
    }

    public void moveArmIntakeRear() {
        turnArmBrakeModeOn();
        armPositionCount = ARM_REAR_INTAKE_POSITION_COUNT;
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_INTAKE_REAR;
    }


    public void moveArmLow() {
        turnArmBrakeModeOn();
        armPositionCount = ARM_LOW_POSITION_COUNT;
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        double pidVal = getPIDValue(ARM_LOW_INTAKE_POSITION_ANGLE);
        armMotorLeft.setPositionPIDFCoefficients(getPIDValue(pidVal));
        armMotorRight.setPositionPIDFCoefficients(pidVal);

        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_LOW;
    }


    public void moveArmMid() {
        turnArmBrakeModeOn();

        armPositionCount = ARM_MID_POSITION_COUNT;
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_ARM_UP;
        double pidVal = getPIDValue(ARM_MID_INTAKE_POSITION_ANGLE);
        armMotorLeft.setPositionPIDFCoefficients(getPIDValue(pidVal));
        armMotorRight.setPositionPIDFCoefficients(pidVal);
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_MID;
    }

    public void moveArmHigh() {
        turnArmBrakeModeOn();

        armPositionCount = ARM_HIGH_POSITION_COUNT;
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        double pidVal = getPIDValue(ARM_HIGH_INTAKE_POSITION_ANGLE);
        armMotorLeft.setPositionPIDFCoefficients(getPIDValue(pidVal));
        armMotorRight.setPositionPIDFCoefficients(pidVal);
        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_HIGH;
    }

    public void moveSSArmSlightlyDown() {
        turnArmBrakeModeOn();
        if (armPositionCount > ARM_DELTA_SLIGHTLY_DOWN_DELTA_COUNT) {
            armPositionCount = armPositionCount - ARM_DELTA_SLIGHTLY_DOWN_DELTA_COUNT; //TODO Amjad : Ensure it is wrapped in an if condition to check the position is not set below 0
        } else {
            armPositionCount = 0;
        }
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        runArmToLevelState = true;
    }

    public void moveSSArmSlightlyUp() {
        turnArmBrakeModeOn();
        if (armPositionCount < ARM_REAR_INTAKE_POSITION_COUNT) {
            armPositionCount = armPositionCount + ARM_DELTA_SLIGHTLY_UP_DELTA_COUNT; //TODO Amjad : Wrap it inside if condition to ensure if does not go beyond REARINTAKEposition. Also sign should be +
        } else {
            armPositionCount = ARM_REAR_INTAKE_POSITION_COUNT;
        }
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        runArmToLevelState = true;

    }

    public ARM_POSITION getArmPosition() {

        return armPosition;
    }


    public int currentEncoderValueLeft() {

        return armMotorLeft.getCurrentPosition();


    }

    public int currentEncoderValueRight() {

        return armMotorRight.getCurrentPosition();


    }

}
