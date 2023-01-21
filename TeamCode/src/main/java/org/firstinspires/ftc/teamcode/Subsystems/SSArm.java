package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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
        ARM_POSITION_CONE_1,
        ARM_POSITION_CONE_2,
        ARM_POSITION_CONE_3,
        ARM_POSITION_CONE_4,
    }


    // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
    //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Encoder Resolution: 537.7 PPR at the Output Shaft
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    private static final double MOTOR_ENCODER_RESOLUTION = 751.8; // 537.7;
    public static double GEAR_RATIO = 2;

    public static int ARM_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = 100; // Determine by experimentation
    public static int ARM_DELTA_SLIGHTLY_UP_DELTA_COUNT = 100; // Determine by experimentation

    // Constants for ARM position based on angle
    // --------------------------------------------------------------------------------------------
    // Angle is measured in direction of needed arm rotation, in degrees
    // Radians will be more accurate, but degrees is close approximation and easier to understand
    //
    // Each revolution of motor moves the shaft 360 deg.
    // In our case, we have gears, and ratio for them is 1:2 (1 rotation of motor turns the shaft 1/2 rotation)
    // Per motor spec sheet, each revolution is 537.7 ticks
    // Therefore, we need 537.7*2 ticks to move the arm 360 deg
    // To calculate the position (ticks) for different angles, we can then use the following
    // formula
    //
    //       (537.7*2)
    //  x =  ---------
    //          360
    //
    // Example, a movement of 45 deg will need ((537.7*2)/360)*45 = 134.42 ticks

    // Note: These are the angles we need to rotate by and not actual physical position angles
    private static final double ARM_FORWARD_INTAKE_POSITION_ANGLE = 0;
    private static final double ARM_LOW_POSITION_ANGLE = 70;
    private static final double ARM_MID_POSITION_ANGLE = 110;
    private static final double ARM_HIGH_POSITION_ANGLE = 135;
    private static final double ARM_CONE1_INTAKE_POSITION_ANGLE = 15;
    private static final double ARM_CONE2_INTAKE_POSITION_ANGLE = 15;
    private static final double ARM_CONE3_INTAKE_POSITION_ANGLE = 10;
    private static final double ARM_CONE4_INTAKE_POSITION_ANGLE = 10;

    // Max we allow arm to go with sightly up functionality
    private static final double ARM_MAX_POSITION_ANGLE = 180;

    // Physical angle of the arm relative to horizontal (0 deg) going counter clockwise
    // This is used to calculate PID constant based on target angle to move
    private static final double ARM_RESTING_PHYSICAL_ANGLE = 250;

    private static final double PID_CONSTANT = 3.8;
    public static double POWER_ARM_UP = .7;

    public ARM_POSITION armPosition = ARM_POSITION.ARM_POSITION_INTAKE_FORWARD;
    public int armPositionCount = 0;
    public boolean runArmToLevelState = false;
    public double motorPowerToRun = POWER_ARM_UP;

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
        armPositionCount = 0; //getPositionFromAngle(ARM_FORWARD_INTAKE_POSITION_ANGLE);

        // This is default PID value - it is then set for each level later
        armMotorLeft.setPositionPIDFCoefficients(PID_CONSTANT);
        armMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);

        armMotorRight.setPositionPIDFCoefficients(PID_CONSTANT);
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
     *
     * @param angle : Target angle the arm will go to. 0 is 3 o'clock, 90 is 12 o'clock.
     *              increasing counter clock wise.
     * @return - Needed P value for PID control
     */
    private double getPIDValue(double angle) {
        // Calculate the physical position angle for PID calculation
        // angle passed in is the angle we want to move by
        /*double calcAngle = ARM_RESTING_PHYSICAL_ANGLE - angle;
        double power = Math.cos(calcAngle) * SSArm.PID_CONSTANT;
        return Math.abs(power);*/

        return PID_CONSTANT;
    }

    /**
     * Calculates the ticks needed to move for a given angle based on encoder resolution
     * (ticks per 360 deg) for the motor
     *
     * @param angle - Angle value to convert
     * @return - Encoder ticks
     */
    private int getPositionFromAngle(double angle) {
        return (int) (((MOTOR_ENCODER_RESOLUTION * GEAR_RATIO) / 360) * angle);
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

    /**
     * Method to run motor to set to the set position
     */
    public void runArmToLevel(double power) {

        if (runArmToLevelState) {
            armMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            armMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            armMotorLeft.setPower(power);
            armMotorRight.setPower(power);
            runArmToLevelState = false;
        } else {
            armMotorLeft.setPower(0.0);
            armMotorRight.setPower(0.0);
        }
    }

    /**
     * Set target position and pid values based on the angle
     *
     * @param targetAngle - Angle we need to go to
     */
    private void setArmTargetPositionValues(double targetAngle) {
        armPositionCount = getPositionFromAngle(targetAngle);
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);

        double pidVal = getPIDValue(targetAngle);
        armMotorLeft.setPositionPIDFCoefficients(pidVal);
        armMotorRight.setPositionPIDFCoefficients(pidVal);

        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
    }

    /**
     * Moves arm to the right height to pick up a cone
     * Used in autonomous mode
     * @param coneCount - Which cone number are we picking
     *                  Cone 0 is pre-loaded
     *                  Cone 1 is top most in the stack
     *                  Cone 2 is next and so on...
     *                  ...
     *                  Cone 5 is last one
     */
    public void moveArmToConeIntakePosition(int coneCount) {
        double targetAngle;
        ARM_POSITION targetPosition;

        switch (coneCount) {
            case 0:
                moveArmIntakeForward();
                return;
            case 1:
                targetPosition = ARM_POSITION.ARM_POSITION_CONE_1;
                targetAngle = ARM_CONE1_INTAKE_POSITION_ANGLE;
                break;
            case 2:
                targetPosition = ARM_POSITION.ARM_POSITION_CONE_2;
                targetAngle = ARM_CONE2_INTAKE_POSITION_ANGLE;
                break;
            case 3:
                targetPosition = ARM_POSITION.ARM_POSITION_CONE_3;
                targetAngle = ARM_CONE3_INTAKE_POSITION_ANGLE;
                break;
            case 4:
                targetPosition = ARM_POSITION.ARM_POSITION_CONE_4;
                targetAngle = ARM_CONE4_INTAKE_POSITION_ANGLE;
                break;
            case 5:
                moveArmIntakeForward();
                return;
            default: {
                // Do nothing
                return;
            }
        }

        turnArmBrakeModeOn();
        setArmTargetPositionValues(targetAngle);
        armPosition = targetPosition;
    }

    /**
     * Move Elevator to level0 Position
     */
    public void moveArmIntakeForward() {
        turnArmBrakeModeOn();
        armPositionCount = 0;
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);

       // double pidVal = getPIDValue(ARM_FORWARD_INTAKE_POSITION_ANGLE);
        armMotorLeft.setPositionPIDFCoefficients(PID_CONSTANT);
        armMotorRight.setPositionPIDFCoefficients(PID_CONSTANT);

        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;

       // setArmTargetPositionValues(ARM_FORWARD_INTAKE_POSITION_ANGLE);
        armPosition = ARM_POSITION.ARM_POSITION_INTAKE_FORWARD;
    }

    public void moveArmLow() {
        turnArmBrakeModeOn();
        setArmTargetPositionValues(ARM_LOW_POSITION_ANGLE);
        armPosition = ARM_POSITION.ARM_POSITION_LOW;
    }

    public void moveArmMid() {
        turnArmBrakeModeOn();
        setArmTargetPositionValues(ARM_MID_POSITION_ANGLE);
        armPosition = ARM_POSITION.ARM_POSITION_MID;
    }

    public void moveArmHigh() {
        turnArmBrakeModeOn();
        setArmTargetPositionValues(ARM_HIGH_POSITION_ANGLE);
        armPosition = ARM_POSITION.ARM_POSITION_HIGH;
    }

    public void moveSSArmSlightlyDown() {
        turnArmBrakeModeOn();
        if (armPositionCount > ARM_DELTA_SLIGHTLY_DOWN_DELTA_COUNT) {
            armPositionCount = armPositionCount - ARM_DELTA_SLIGHTLY_DOWN_DELTA_COUNT;
        } else {
            armPositionCount = 0;
        }
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        runArmToLevelState = true;
    }

    public void moveSSArmSlightlyUp() {
        turnArmBrakeModeOn();
        int maxPosition = getPositionFromAngle(ARM_MAX_POSITION_ANGLE);

        if (armPositionCount < maxPosition) {
            armPositionCount = armPositionCount + ARM_DELTA_SLIGHTLY_UP_DELTA_COUNT;
        } else {
            armPositionCount = maxPosition;
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
