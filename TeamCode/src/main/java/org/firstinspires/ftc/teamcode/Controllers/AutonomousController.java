package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSBucket;
import org.firstinspires.ftc.teamcode.SubSystems.SSElevator;
import org.firstinspires.ftc.teamcode.SubSystems.SSIntake;
import org.firstinspires.ftc.teamcode.SubSystems.SSSpinner;

/**
 * Defenition of the AutoControl Class <BR>
 *
 * HzAutoControl consists of methods to control the robot subsystems in autonomous mode <BR>
 * This is set up as a state machine. And replicates all commands as in the gamepad <BR>
 * 
 */

public class AutonomousController {

    //Create gamepad object reference to connect to gamepad1
    public DriveTrain driveTrain;
    public SSIntake ssIntake;
    public SSElevator ssElevator;
    public SSBucket ssBucket;
    public SSSpinner ssSpinner;

    public Pose2d startPose = GameField.BLUE_STARTPOS_1;

    /**
     * Constructor for HzGamepad1 class that extends gamepad.
     * Assign the gamepad1 given in OpMode to the gamepad used here.
     *
     * TODO: Add more subsystems in declaration
     */
    public AutonomousController(DriveTrain driveTrain,
                                SSIntake ssIntake,
                                SSElevator ssElevator,
                                SSBucket ssBucket,
                                SSSpinner ssSpinner) {
        this.driveTrain = driveTrain;
        this.ssIntake = ssIntake;
        this.ssElevator = ssElevator;
        this.ssBucket = ssBucket;
        this.ssSpinner = ssSpinner;
        //TODO: Add more subsystems
    }


    /**
     * Main control function that calls the runs of each of the subsystems in sequence
     */
    public void runAutoControl(){
        int counter = 0;
        while (counter < 5) {

            //TODO_SS : add runIntakeControl(), runBucketControl(), runSpinnerControl(), runElevatorControl()
            runSubsystem1Control();
            //TODO: Add runSubsystemControl functions for each subsystem
            counter++;
        }
    }

    //------------code starts here
    //TODO_SS: Add for all subsystems

    /***   START of  Autocontrol State and function for SSIntake
     ****/

    enum AUTO_SSINTAKE_STATE{
        START,
        STOP,
    }
    AUTO_SSINTAKE_STATE autossIntakeState = AUTO_SSINTAKE_STATE.STOP;

//bucket class functions
    public void autoBucketSetToDrop() {

        if (ssBucket.bucketServoState != SSBucket.BUCKET_SERVO_STATE.DROP_POSITION) {
            ssBucket.bucketServo.setPosition(SSBucket.BUCKET_SERVO_DROP_POSITION);
            ssBucket.bucketServoState = SSBucket.BUCKET_SERVO_STATE.DROP_POSITION;
        }
    }

    public void autoBucketSetToCollect() {

        if (ssBucket.bucketServoState != SSBucket.BUCKET_SERVO_STATE.COLLECT_POSITION) {
            ssBucket.bucketServo.setPosition(SSBucket.BUCKET_SERVO_COLLECT_POSITION);
            ssBucket.bucketServoState = SSBucket.BUCKET_SERVO_STATE.COLLECT_POSITION;
        }
    }

    public void autoBucketSetToTransport() {

        if (ssBucket.bucketServoState != SSBucket.BUCKET_SERVO_STATE.TRANSPORT_POSITION) {
            ssBucket.bucketServo.setPosition(SSBucket.BUCKET_SERVO_TRANSPORT_POSITION);
            ssBucket.bucketServoState = SSBucket.BUCKET_SERVO_STATE.TRANSPORT_POSITION;
        }
    }

// intake class functions
    public void autoStartAutoSSIntake(){
        autossIntakeState = AUTO_SSINTAKE_STATE.START;
        runAutoControl();
    }

    public void autoStartForwardSSIntakeMotor() {
        if (ssIntake.SSIntakeMotorState != SSIntake.SSINTAKE_MOTOR_STATE.RUNNING) {
            ssIntake.startForwardSSIntakeMotor();
            ssIntake.SSIntakeMotorState = SSIntake.SSINTAKE_MOTOR_STATE.RUNNING;
        }
    }

    public void autoStartReverseSSIntakeMotor() {
        if(ssIntake.SSIntakeMotorState != SSIntake.SSINTAKE_MOTOR_STATE.REVERSING) {
            ssIntake.startReverseSSIntakeMotor();
            ssIntake.SSIntakeMotorState = SSIntake.SSINTAKE_MOTOR_STATE.REVERSING;

        }
    }

    public SSIntake.SSINTAKE_MOTOR_STATE autoGetSSIntakeMotorState() {
        return ssIntake.SSIntakeMotorState;
    }

    // elevator class functions

    public void autoInitElevator(){
        ssElevator.elevatorMotor.setPositionPIDFCoefficients(5.0); //  TODO : Determine by experimentation
        ssElevator.elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);  //TODO : Determine by experimentation
        ssElevator.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ssElevator.resetElevator();
        ssElevator.moveElevatorLevel0();
        ssElevator.turnElevatorBrakeModeOn();
    }

    public void autoResetElevator(){
        DcMotor.RunMode runMode = ssElevator.elevatorMotor.getMode();
        ssElevator.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ssElevator.elevatorMotor.setMode(runMode);
    }

    public void autoTurnElevatorBrakeModeOn(){
        ssElevator.elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void autoTurnElevatorBrakeModeOff(){
        ssElevator.elevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void autoRunElevatorToLevel(double power){
        ssElevator.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (ssElevator.runElevatorToLevelState || ssElevator.elevatorMotor.isBusy() ){
            ssElevator.elevatorMotor.setPower(power);
            ssElevator.runElevatorToLevelState = false;
        } else {
            ssElevator.elevatorMotor.setPower(0.0);
        }
    }

    public void autoMoveElevatorLevel0() {
        ssElevator.turnElevatorBrakeModeOff();
        ssElevator.elevatorPositionCount = ssElevator.ELEVATOR_LEVEL0_POSITION_COUNT + ssElevator.baselineEncoderCount;
        ssElevator.elevatorMotor.setTargetPosition(ssElevator.elevatorPositionCount);
        ssElevator.motorPowerToRun = SSElevator.POWER_NO_CARGO;
        ssElevator.runElevatorToLevelState = true;
        ssElevator.elevatorPosition = SSElevator.ELEVATOR_POSITION.LEVEL_0;
    }

    public void autoMoveElevatorLevel1() {
        ssElevator.turnElevatorBrakeModeOn();
        ssElevator.elevatorPositionCount = SSElevator.ELEVATOR_LEVEL1_POSITION_COUNT + SSElevator.baselineEncoderCount;
        ssElevator.elevatorMotor.setTargetPosition(ssElevator.elevatorPositionCount);
        ssElevator.motorPowerToRun = SSElevator.POWER_WITH_CARGO;
        ssElevator.runElevatorToLevelState = true;
        ssElevator.elevatorPosition = SSElevator.ELEVATOR_POSITION.LEVEL_1;
    }

    public void autoMoveElevatorLevel2() {
        ssElevator.turnElevatorBrakeModeOn();
        ssElevator.elevatorPositionCount = SSElevator.ELEVATOR_LEVEL2_POSITION_COUNT + SSElevator.baselineEncoderCount;
        ssElevator.elevatorMotor.setTargetPosition(ssElevator.elevatorPositionCount);
        ssElevator.motorPowerToRun = SSElevator.POWER_WITH_CARGO;
        ssElevator.runElevatorToLevelState = true;
        ssElevator.elevatorPosition = SSElevator.ELEVATOR_POSITION.LEVEL_2;
    }

    public void autoMoveElevatorLevel3() {
        ssElevator.turnElevatorBrakeModeOn();
        ssElevator.elevatorPositionCount = SSElevator.ELEVATOR_LEVEL3_POSITION_COUNT + SSElevator.baselineEncoderCount;
        ssElevator.elevatorMotor.setTargetPosition(ssElevator.elevatorPositionCount);
        ssElevator.motorPowerToRun = SSElevator.POWER_WITH_CARGO;
        ssElevator.runElevatorToLevelState = true;
        ssElevator.elevatorPosition = SSElevator.ELEVATOR_POSITION.LEVEL_3;
    }

    public SSElevator.ELEVATOR_POSITION autoGetElevatorPosition() {

        return ssElevator.elevatorPosition;
    }

// spinner class function

    public void autoStartClockwiseSSSPinnerMotor() {
        if (ssSpinner.SSSpinnerMotorState != SSSpinner.SSSPINNER_MOTOR_STATE.CLOCKWISE) {
            ssSpinner.runSSSpinnerMotor(DcMotor.Direction.FORWARD, ssSpinner.SSSpinnerMotorPower1);
            ssSpinner.SSSpinnerMotorState = SSSpinner.SSSPINNER_MOTOR_STATE.CLOCKWISE;
        }
    }

    public void autoStopSSSpinnerMotor() {
        if (ssSpinner.SSSpinnerMotorState != SSSpinner.SSSPINNER_MOTOR_STATE.STOPPED) {
            ssSpinner.runSSSpinnerMotor(DcMotor.Direction.FORWARD, 0.0);
            ssSpinner.SSSpinnerMotorState = SSSpinner.SSSPINNER_MOTOR_STATE.STOPPED;
        }
    }

    public void autoStartAntiClockwiseSSSpinnerMotor() {
        if (ssSpinner.SSSpinnerMotorState != SSSpinner.SSSPINNER_MOTOR_STATE.ANTICLOCKWISE) {
            ssSpinner.runSSSpinnerMotor(DcMotor.Direction.REVERSE, ssSpinner.SSSpinnerMotorPower1);
            ssSpinner.SSSpinnerMotorState = SSSpinner.SSSPINNER_MOTOR_STATE.ANTICLOCKWISE;
        }
    }

    public void autoRunSSSpinnerMotor(DcMotor.Direction direction, double power) {
        ssSpinner.SSSpinnerMotor.setDirection(direction);
        ssSpinner.SSSpinnerMotor.setPower(power);
    }

    public SSSpinner.SSSPINNER_MOTOR_STATE autoGetSSSpinnerMotorState() {
        return ssSpinner.SSSpinnerMotorState;
    }

    //TODO_SS:
    /***   END of   Autocontrol State and function for SSSPinner
    ****/

    /***   START of  Autocontrol State and function for SSSPinner
     ****/

    enum AUTO_SUBSYSTEM2_STATE{
        START,
        STOP,
        //TODO:Update Subsystem states as appropriate
    }
    AUTO_SUBSYSTEM2_STATE autoSubsystem2State = AUTO_SUBSYSTEM2_STATE.STOP;



    //TODO: Add states for each Subsystem


    public void runSubsystem2Control(){

        if (autoSubsystem2State == AUTO_SUBSYSTEM2_STATE.START){
            //TODO: Add state setting code for Subsystem1
           //start spinner
        }

        if (autoSubsystem2State == AUTO_SUBSYSTEM2_STATE.STOP){
            //TODO: Add state setting code for Subsystem1
             //stop spinner
        }
    }

    /***   END of   Autocontrol State and function for SSPinner
     ****/

    //TODO_SS:

    /***   START of  Autocontrol State and function for SSBucket
     ****/

    enum AUTO_SUBSYSTEM3_STATE{
        START,
        STOP,
        //TODO:Add bucket positions
    }
    AUTO_SUBSYSTEM3_STATE autoSubsystem3State = AUTO_SUBSYSTEM3_STATE.STOP;



    //TODO: Add states for each Subsystem


    public void runSubsystem3Control(){

        if (autoSubsystem3State == AUTO_SUBSYSTEM3_STATE.START){
            //TODO: Add state setting code for Subsystem1
            /* Set state for subsystem - Example
            acHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = false;
            acHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
            acHzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.COLLECT;
             */
        }

        if (autoSubsystem3State == AUTO_SUBSYSTEM3_STATE.STOP){
            //TODO: Add state setting code for Subsystem1
             /* Set state for subsystem - Example
            acHzIntakeUltimateGoal.intakeButtonState = HzIntakeUltimateGoal.INTAKE_BUTTON_STATE.OFF;
              */
        }
    }

    /***   END of   Autocontrol State and function for SSBucket
     ****/

    //TODO_SS:

    /***   START of  Autocontrol State and function for SSElevator
     ****/

    enum AUTO_SUBSYSTEM4_STATE{
        START,
        STOP,
        //TODO:Add Levels
    }
    AUTO_SUBSYSTEM4_STATE autoSubsystem4State = AUTO_SUBSYSTEM4_STATE.STOP;



    //TODO: Add states for each Subsystem

    /** TODO: Add move functions for all levels
    public void moveAutoElevatorLevel0(){
        autoElevatorState = AUTO_ELEVATOR_STATE.LEVEL_0;
        runAutoControl();
    }
     **/


    public void runSubsystem4Control(){

        //add if condition to check elevator state is in LEVEL 0 then stop intake motor and move bucket to collect
        /*If (elevatorState == Level 0){
            ssintake.stopIntake
            ssBucket.setToCollect
         */

        //Add switch statement for all ohter levels
        /*** swtich(evelatorState){
            case LEVEL1 : elevator.movetoLevel1;
                            break;
            case LEVEL2:
            case LEVEL3:
         }
         ****/


        if (autoSubsystem4State == AUTO_SUBSYSTEM4_STATE.START){
            //TODO: Add state setting code for Subsystem4
            /* Set state for subsystem - Example
            acHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = false;
            acHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
            acHzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.COLLECT;
             */
        }

        if (autoSubsystem4State == AUTO_SUBSYSTEM4_STATE.STOP){
            //TODO: Add state setting code for Subsystem4
             /* Set state for subsystem - Example
            acHzIntakeUltimateGoal.intakeButtonState = HzIntakeUltimateGoal.INTAKE_BUTTON_STATE.OFF;
              */
        }
    }

    /***   END of   Autocontrol State and function for SSElevator
     ****/

    //--------Template for each subsytemis below..copy and paste above and edit-------------------------------------------------
    // Define and delcare autonomous states
    // Example
    enum AUTO_SUBSYSTEM1_STATE{
        START,
        STOP,
        //TODO:Update Subsystem states as appropriate
    }
    AUTO_SUBSYSTEM1_STATE autoSubsystem1State = AUTO_SUBSYSTEM1_STATE.STOP;



    //TODO: Add states for each Subsystem

    /**
     * run Intake Control State machine response
     * Also deactivate Launch readiness when Intake is started
     */
    public void runSubsystem1Control(){

        if (autoSubsystem1State == AUTO_SUBSYSTEM1_STATE.START){
            //TODO: Add state setting code for Subsystem1
            /* Set state for subsystem - Example
            acHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = false;
            acHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
            acHzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.COLLECT;
             */
        }

        if (autoSubsystem1State == AUTO_SUBSYSTEM1_STATE.STOP){
            //TODO: Add state setting code for Subsystem1
             /* Set state for subsystem - Example
            acHzIntakeUltimateGoal.intakeButtonState = HzIntakeUltimateGoal.INTAKE_BUTTON_STATE.OFF;
              */
        }
    }

    //TODO: Add more run Subsystem Control functions

}