package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSArm;
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
    public SSArm ssArm;

    public Pose2d startPose = GameField.ORIGINPOSE;

    /**
     * Constructor for HzGamepad1 class that extends gamepad.
     * Assign the gamepad1 given in OpMode to the gamepad used here.
     *
     */
    public AutonomousController(DriveTrain driveTrain,
                                SSIntake ssIntake,
                                SSElevator ssElevator,
                                SSBucket ssBucket,
                                SSSpinner ssSpinner,
                                SSArm ssArm) {
        this.driveTrain = driveTrain;
        this.ssIntake = ssIntake;
        this.ssElevator = ssElevator;
        this.ssBucket = ssBucket;
        this.ssSpinner = ssSpinner;
        this.ssArm = ssArm;
    }


    /**
     * Main control function that calls the runs of each of the subsystems in sequence
     */
    public void runAutoControl(){
        int counter = 0;
        while (counter < 5) {
            runAutoSSIntake();
            runAutoSSElevator();
            runAutoSSBucket();
            runAutoSSSpinner();
            runAutoSSArm();
            counter++;
        }
    }

    /**
     * intake class functions
     */
    enum AUTO_SSINTAKE_STATE{
        RUNNING,
        STOPPED,
    }
    AUTO_SSINTAKE_STATE autoSSIntakeState = AUTO_SSINTAKE_STATE.STOPPED;

    public void autoStartAutoSSIntake(){
        autoSSIntakeState = AUTO_SSINTAKE_STATE.RUNNING;
        runAutoControl();
    }

    public void autoStopSSIntake(){
        autoSSIntakeState = AUTO_SSINTAKE_STATE.STOPPED;
        runAutoControl();
    }

    public void runAutoSSIntake() {
        if (autoSSIntakeState == AUTO_SSINTAKE_STATE.RUNNING &&
                ssElevator.getElevatorPosition() == SSElevator.ELEVATOR_POSITION.LEVEL_0) {
            if (ssBucket.bucketServoState != SSBucket.BUCKET_SERVO_STATE.COLLECT_POSITION) {
                ssBucket.setToCollect();
            }
            ssIntake.startForwardSSIntakeMotor();
        } else { //autossIntakeState == AUTO_SSINTAKE_STATE.STOPPED
            ssIntake.stopSSIntakeMotor();
        }
    }

    public AUTO_SSINTAKE_STATE autoGetSSIntakeState() {
        return autoSSIntakeState;
    }
    /***** End of Intake functions ****/

    /**
     * elevator class functions
     */
    enum  AUTO_SSELEVATOR_STATE{
        LEVEL_0,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
    }
    AUTO_SSELEVATOR_STATE autoSSElevatorState = AUTO_SSELEVATOR_STATE.LEVEL_0;

    public void autoSSElevatorSetToLevel0(){
        autoSSElevatorState = AUTO_SSELEVATOR_STATE.LEVEL_0;
        runAutoControl();
    }

    public void autoSSElevatorSetToLevel1(){
        autoSSElevatorState = AUTO_SSELEVATOR_STATE.LEVEL_1;
        runAutoControl();
    }

    public void autoSSElevatorSetToLevel2(){
        autoSSElevatorState = AUTO_SSELEVATOR_STATE.LEVEL_2;
        runAutoControl();
    }

    public void autoSSElevatorSetToLevel3(){
        autoSSElevatorState = AUTO_SSELEVATOR_STATE.LEVEL_3;
        runAutoControl();
    }

    public void runAutoSSElevator() {
        if (autoSSElevatorState == AUTO_SSELEVATOR_STATE.LEVEL_0){
            ssElevator.moveElevatorLevel0();
            autoSSBucketState = AUTO_SSBUCKET_STATE.COLLECT;

        } else {
            autoSSIntakeState = AUTO_SSINTAKE_STATE.STOPPED;
            //autoMagazineState = AUTO_MAGAZINE_STATE.TRANSPORT;
            switch (autoSSElevatorState){
                case LEVEL_1:
                    ssElevator.moveElevatorLevel1();
                    break;
                case LEVEL_2:
                    ssElevator.moveElevatorLevel2();
                    break;
                case LEVEL_3:
                    ssElevator.moveElevatorLevel3();
                    break;
            }
        }

        if (ssElevator.runElevatorToLevelState){
            ssElevator.runElevatorToLevel(ssElevator.motorPowerToRun);
        }
    }

    public AUTO_SSELEVATOR_STATE autoGetSSElevatorState() {
        return autoSSElevatorState;
    }
    /**** End of Elevator functions ***/

    /****
     * Bucket functions
     */
    enum AUTO_SSBUCKET_STATE{
        COLLECT,
        TRANSPORT,
        DROP,
    }
    AUTO_SSBUCKET_STATE autoSSBucketState = AUTO_SSBUCKET_STATE.COLLECT;

    public void autoBucketSetToDrop() {
        autoSSBucketState = AUTO_SSBUCKET_STATE.DROP;
        runAutoControl();
    }

    public void autoBucketSetToCollect() {
        autoSSBucketState = AUTO_SSBUCKET_STATE.COLLECT;
        runAutoControl();
    }

    public void autoBucketSetToTransport() {
        autoSSBucketState = AUTO_SSBUCKET_STATE.TRANSPORT;
        runAutoControl();
    }

    public void runAutoSSBucket() {
        /*if (ssBucket.magazineColorSensor instanceof SwitchableLight) {
            if (ssElevator.getElevatorPosition() == SSElevator.ELEVATOR_POSITION.LEVEL_0 &&
                    ssBucket.getMagazineColorSensorState() == SSBucket.MAGAZINE_COLOR_SENSOR_STATE.EMPTY) {
                ((SwitchableLight) ssBucket.magazineColorSensor).enableLight(true);
            } else {
                ((SwitchableLight) ssBucket.magazineColorSensor).enableLight(false);
            }
        }*/

        switch (autoSSBucketState){
            case TRANSPORT:
                ssBucket.setToTransport();
                break;
            case DROP :
                ssBucket.setToDrop();
                break;
            case COLLECT:
                ssBucket.setToCollect();
                break;
        }
    }

    public AUTO_SSBUCKET_STATE autoGetSSBucketState() {
        return autoSSBucketState;
    }
    /**** End of Bucket functions ***/

    /**
     * spinner class function
     */

    public enum AUTO_SSSPINNER_STATE {
        CLOCKWISE,
        ANTICLOCKWISE,
        STOPPED,
    }

    public AUTO_SSSPINNER_STATE autoSSSpinnerState = AUTO_SSSPINNER_STATE.STOPPED;

    public void runAutoSSSpinner() {
        switch (autoSSSpinnerState) {
            case ANTICLOCKWISE:
                ssSpinner.startAntiClockwiseSSSpinnerMotor();
                break;
            case CLOCKWISE:
                ssSpinner.startClockwiseSSSPinnerMotor();
                break;
            case STOPPED:
                ssSpinner.stopSSSpinnerMotor();
                break;
        }
    }

    public AUTO_SSSPINNER_STATE autoGetSSSpinnerState() {
        return autoSSSpinnerState;
    }
    /***   END of   Autocontrol State and function for SSSPinner  ****/

    /**
     * runAutoArm() function
     */
    enum AUTO_SSARM_STATE {
        PICKUP,
        PARKED,
    }
    AUTO_SSARM_STATE autoSSArmState = AUTO_SSARM_STATE.PARKED;

    public void autoSSArmSetToPickup(){
        autoSSArmState = AUTO_SSARM_STATE.PICKUP;
        runAutoControl();
    }

    public void autoSSArmSetToPark(){
        autoSSArmState = AUTO_SSARM_STATE.PARKED;
        runAutoControl();
    }
    public enum AUTO_SSGRIP_STATE {
        OPEN,
        CLOSED,
    }
    AUTO_SSGRIP_STATE autoSSGripState = AUTO_SSGRIP_STATE.CLOSED;

    public void autoSSGripSetToOpen(){
        autoSSGripState = AUTO_SSGRIP_STATE.OPEN;
        runAutoControl();
    }

    public void autoSSGripSetToClose(){
        autoSSGripState = AUTO_SSGRIP_STATE.CLOSED;
        runAutoControl();
    }

    public void runAutoSSArm() {
        if(autoSSArmState == AUTO_SSARM_STATE.PARKED){
            ssArm.moveArmParked();
        } else {
            ssArm.moveArmPickup();
        }

        if (ssArm.runArmToLevelState) {
            ssArm.runArmToLevel(ssArm.POWER_ARM_UP);
        }

        if(autoSSGripState == AUTO_SSGRIP_STATE.OPEN){
            ssArm.setGripOpen();
        } else{
            ssArm.setGripClose();
        }

    }

}