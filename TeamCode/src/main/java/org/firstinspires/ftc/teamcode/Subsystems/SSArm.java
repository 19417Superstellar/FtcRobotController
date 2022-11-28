package org.firstinspires.ftc.teamcode.Subsystems;

        import com.qualcomm.robotcore.hardware.DcMotor; //TODO Amjad : Remove DCMotor class only use DCMotorEx class. 
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple; //TODO Amjad : Remove DCMotorSimple class only use DCMotorEx class. 
        import com.qualcomm.robotcore.hardware.HardwareMap;
public class SSArm {


    public DcMotorEx armMotorRight;
    public DcMotorEx armMotorLeft;
    //add 2 motors left and right
    //Gobilda (enter motor details later
    //Encoder count : enter details to be done


    public enum ARM_POSITION {
       ARM_POSITION_INTAKE_FORWARD,
        ARM_POSITION_INTAKE_REAR,
        ARM_POSITION_MID,
        ARM_POSITION_LOW,
        ARM_POSITION_HIGH
//TODO : Amjad Add a RANDOM state to be used for slightly high or low situations
    }


    // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
    //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Encoder Resolution: 537.7 PPR at the Output Shaft
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    //public static double ENCODER_VALUE = 537.7;
    public static int baselineEncoderCount = 0; //TODO : Amjad Can removbe this, not used.
   // public static int ARM_BASELINE_POSITION_COUNT = 0;
    //Use testarm teleop to verify if armMotorLeft and armMotorRight use the same encoder values
    //if not, add different constants for the two motors
    //public static int ARM_CUP_POSITION_COUNT = 375;//  TODO : Determine by experimentation
    public static int ARM_LOW_POSITION_COUNT=0; //TODO : Determine by experimentation //TODO : Amjad This should be around 200, assuming the starting as FORWARD Intake Position
    public static int ARM_MID_POSITION_COUNT=200; //TODO : Determine by experimentation //TODO : Amjad This should be around 300
    public static int ARM_HIGH_POSITION_COUNT=450; //TODO : Determine by experimentation 
    public static int ARM_FORWARD_INTAKE_POSITION_COUNT=800; //TODO : Determine by experimentation //TODO : Amjad This should be 0, suggest to keep the arm forward while starting. Keep grip open to keep withing 18"
    public static int ARM_REAR_INTAKE_POSITION_COUNT=0; //TODO : Determine by experimentation //TODO : Amjad This value should be around 800.
    public static int ARM_DELTA_SLIGHTLY_DOWN_DELTA_COUNT=40; //TODO : Determine by experimentation
    public static int ARM_DELTA_SLIGHTLY_UP_DELTA_COUNT=40; //TODO : Determine by experimentation //TODO : Amjad UP and DOWN can be same count, since it is a symmetrical robot, can eliminate one of these.
    //add count positions for different junctions
    //MAX 2200


    public static double POWER_ARM_UP = 0.2; //TODO : Amjad Power may be too less to operate this. Keep around 0.5 to 0.8


    public ARM_POSITION armPosition = ARM_POSITION.ARM_POSITION_INTAKE_FORWARD;
    //public ARM_POSITION previousPosition = ARM_POSITION.ARM_POSITION_INTAKE_FORWARD;


    public int armPositionCount = ARM_FORWARD_INTAKE_POSITION_COUNT;



    public SSArm(HardwareMap hardwareMap) {
        armMotorLeft = hardwareMap.get(DcMotorEx.class,"arm_motor_left");
        armMotorRight = hardwareMap.get(DcMotorEx.class,"arm_motor_right");
        initArm();
    }

    /**
     * Initialization for the Arm
     */
    public void initArm(){
        resetArm();
        turnArmBrakeModeOff();
        armMotorLeft.setPositionPIDFCoefficients(5.0);
        armMotorLeft.setTargetPosition(ARM_FORWARD_INTAKE_POSITION_COUNT); //TODO : Amjad You cannot set position right at init. It will swing the arm when setPower is called. Remove statement
        armMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotorRight.setPositionPIDFCoefficients(5.0);
        armMotorRight.setTargetPosition(ARM_FORWARD_INTAKE_POSITION_COUNT); //TODO : Amjad You cannot set position right at init. It will swing the arm when setPower is called. Remove statement
        armMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armPosition = ARM_POSITION.ARM_POSITION_INTAKE_FORWARD;
    }


    public void resetArm(){
        //DcMotor.RunMode runMode = elevatorMotor.getMode();
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //armMotor.setMode(runMode);
    }

    /**
     * Method to set Elevator brake mode to ON when Zero (0.0) power is applied. <BR>
     * To be used when arm is above groundlevel
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOn(){
        armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * Method to set Elevator brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOff(){
        armMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        armMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    }

    public boolean runArmToLevelState = false;
    public double motorPowerToRun = POWER_ARM_UP;

    /**
     * Method to run motor to set to the set position
     */
    public void runArmToLevel(double power){
        armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //TODO: Amjad Move this to inside of the if condition with runArmToLevelState
        armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION); //TODO: Amjad Move this to inside of the if condition with runArmToLevelState
        if (runArmToLevelState /*|| armMotorLeft.isBusy()*/ ){
            armMotorLeft.setPower(power);
            armMotorRight.setPower(power);
            runArmToLevelState = false;
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
        armPositionCount = ARM_FORWARD_INTAKE_POSITION_COUNT + baselineEncoderCount; //TODO: Amjad : Remove baselineEncoderCount as it is not determined
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_INTAKE_FORWARD;
    }

    public void moveArmIntakeRear() {
        turnArmBrakeModeOn();
        armPositionCount = ARM_REAR_INTAKE_POSITION_COUNT + baselineEncoderCount;
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_INTAKE_REAR;
    }


    public void moveArmLow() {
        turnArmBrakeModeOn();
        armPositionCount = ARM_LOW_POSITION_COUNT + baselineEncoderCount;
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_LOW;
    }


    public void moveArmMid() {
        turnArmBrakeModeOn();

        armPositionCount = ARM_MID_POSITION_COUNT + baselineEncoderCount + 30;
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_MID;
    }
    public void moveArmHigh() {
        turnArmBrakeModeOn();

        armPositionCount = ARM_HIGH_POSITION_COUNT + baselineEncoderCount + 30;
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_ARM_UP;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_HIGH;
    }

    public void moveSSArmSlightlyDown() {
            turnArmBrakeModeOn();
            armPositionCount = armPositionCount - ARM_DELTA_SLIGHTLY_DOWN_DELTA_COUNT; //TODO Amjad : Ensure it is wrapped in an if condition to check the position is not set below 0
            armMotorLeft.setTargetPosition(armPositionCount);
            armMotorRight.setTargetPosition(armPositionCount);
            runArmToLevelState = true;
            armPositionCount = ARM_LOW_POSITION_COUNT; //TODO Amjad : State shouldbe set here to RANDOM, not count
    }

    public void moveSSArmSlightlyUp() {
        turnArmBrakeModeOn();
        armPositionCount = armPositionCount - ARM_DELTA_SLIGHTLY_UP_DELTA_COUNT; //TODO Amjad : Wrap it inside if condition to ensure if does not go beyond REARINTAKEposition. Also sign should be +
        armMotorLeft.setTargetPosition(armPositionCount);
        armMotorRight.setTargetPosition(armPositionCount);
        runArmToLevelState = true;
        armPositionCount = ARM_HIGH_POSITION_COUNT; //TODO Amjad : State shouldbe set here to RANDOM, not count
    }






    //function to determinie POWER_WITH_CARGO or POWER_WITH_NO_CARGO

    public ARM_POSITION getArmPosition() {

        return armPosition;
    }



    public int currentEncoderValueLeft(){

        return armMotorLeft.getCurrentPosition();


    }

    public int currentEncoderValueRight(){

        return armMotorRight.getCurrentPosition();


    }

}
