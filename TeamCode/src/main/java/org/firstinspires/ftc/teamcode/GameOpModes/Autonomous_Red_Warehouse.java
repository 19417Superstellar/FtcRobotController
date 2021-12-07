package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.AutonomousController;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSBucket;
import org.firstinspires.ftc.teamcode.SubSystems.SSElevator;
import org.firstinspires.ftc.teamcode.SubSystems.SSIntake;
import org.firstinspires.ftc.teamcode.SubSystems.SSSpinner;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;


/**
 * Path and actions for autonomous mode starting from Red Warehouse position
 * 1.	Robot starts position with bucket in transport state in Level 1 with pre-loaded box in it.
 * 2.	Call Vision function to detect duck/Team Marker position and set level to drop pre-loaded box
 * 3.	Call roadrunner function to move to duck/Team Marker position
 * 4.	Call arm function to pick up duck / Team Marker
 * 5.	Call roadrunner function move to Alliance Shipping Hub
 * 6.	Call elevator function to raise to correct level drop preloaded box
 * 7.	Call bucket function to drop preloaded box
 * 8.	Call bucket function to change to transport state
 * 9.	Call elevator function to move to Level 1 position
 * 10.	Call roadrunner function to move to parking position in storage area through gap
 */
@Autonomous(name = "Autonomous_Red_Warehouse", group = "00-Autonomous" , preselectTeleOp = "SSTeleOp")
public class Autonomous_Red_Warehouse extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadController gamepadController;
    public AutonomousController autonomousController;
    public DriveTrain driveTrain;
    public SSSpinner ssSpinner;
    public SSBucket ssBucket;
    public SSElevator ssElevator;
    public SSIntake ssIntake;
    //public SSArm ssArm;

    //TODO: Replace name of Subsystem1 and Declare more subsystems

    public Vision vision;
    public static final Pose2d RED_WAREHOUSE_STARTPOS =  new Pose2d(61,7,Math.toRadians(0));
    public Pose2d startPose = RED_WAREHOUSE_STARTPOS;

    boolean parked = false ;
    boolean autonomousStarted = false;

    public Vision.ACTIVE_WEBCAM activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
    public GameField.VISION_IDENTIFIED_TARGET targetZone = GameField.VISION_IDENTIFIED_TARGET.LEVEL1;

    double af = GameField.ALLIANCE_FACTOR;

    Trajectory traj;


    @Override
    public void runOpMode() throws InterruptedException {
        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        ssSpinner = new SSSpinner(hardwareMap);
        ssBucket = new SSBucket(hardwareMap);
        ssElevator = new SSElevator(hardwareMap);
        ssIntake = new SSIntake(hardwareMap);

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1,gamepad2, driveTrain, ssIntake, ssElevator, ssBucket, ssSpinner);
        autonomousController = new AutonomousController(driveTrain, ssIntake, ssElevator, ssBucket, ssSpinner);

        //Key Pay inputs to select Game Plan;
        vision = new Vision(hardwareMap, activeWebcam);
        af = GameField.ALLIANCE_FACTOR;

        // Initiate Camera on Init.
        vision.activateVuforiaTensorFlow();

        // 1.	Robot starts position with bucket in transport state in Level 1 with pre-loaded box in it.
        startPose = RED_WAREHOUSE_STARTPOS;
        driveTrain.getLocalizer().setPoseEstimate(startPose);

        //Robot starts with Elevator in Collect State, with preloaded box
        //On Init, Elevator moves to Level 1, Bucket moves to transport
        autonomousController.autoSSElevatorSetToLevel1();
        autonomousController.autoBucketSetToTransport();
        autonomousController.runAutoControl();

        telemetry.addData("Start Pose : ", "BLUE_WAREHOUSE_STARTPOS");
        telemetry.addData("Waiting for start to be pressed.","Robot is ready!");
        telemetry.update();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            // 2.	Call Vision function to detect duck/Team Marker position and set level to drop pre-loaded box
            //Run Vuforia Tensor Flow
            targetZone = vision.runVuforiaTensorFlow();

            if (!parked){
                autonomousController.runAutoControl();
            }

            if (DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            //Game Play is pressed
            while (opModeIsActive() && !isStopRequested() && !parked) {

                vision.deactivateVuforiaTensorFlow();
                runAutoRedWarehouse();

                //Move to Launching Position
                parked = true;

                //Write last position to static class to be used as initial position in TeleOp
                GameField.currentPose = driveTrain.getPoseEstimate();
                GameField.poseSetInAutonomous = true;

                if (DEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }
            }

        }

        //Write last position to static class to be used as initial position in TeleOp
        GameField.currentPose = driveTrain.getPoseEstimate();
        GameField.poseSetInAutonomous = true;    }

    public void runAutoRedWarehouse(){
        //Logic for waiting
        safeWait(100);

        // 3.	Call roadrunner function to move to duck/Team Marker position
        //Move arm to Pickup Capstone level and open Grip
        //TODO: Arm function
        //moveMajorArmToPickupAndOpenClaw();
        safeWait(2000);

        //Move forward to Capstone Pickup Position
        switch (targetZone) {
            case LEVEL1:
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(49, 9, Math.toRadians(180)))
                        .build();
                break;
            case LEVEL2:
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(48, 11, Math.toRadians(-30)))
                        .build();
                break;
            case LEVEL3:
            case UNKNOWN:
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(47, 13, Math.toRadians(-60)))
                        .build();
                break;
        }
        driveTrain.followTrajectory(traj);

        // 4.	Call arm function to pick up duck / Team Marker
        //TODO: Arm function
        //moveMajorArmToParkingAfterClosingClaw();

        // 5.	Call roadrunner function move to Alliance Shipping Hub
        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(47, 13, Math.toRadians(-60)))
                .build();
        driveTrain.followTrajectory(traj);

        // 6.	Call elevator function to raise to correct level drop preloaded box
        // 7.	Call bucket function to drop preloaded box
        // 8.	Call bucket function to change to transport state
        dropBoxToLevel();

        // 9.	Call elevator function to move to Level 1 position
        safeWait(1000);
        moveElevatorToLevel1();

        // 10.	Call roadrunner function to move to parking position in storage area through gap
        traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                .splineTo(new Vector2d(-60, 12), Math.toRadians(90))
                .splineTo(new Vector2d(-60, 42), Math.toRadians(90))
                .build();
        driveTrain.followTrajectory(traj);
    }

    /**
     * Safe method to wait so that stop button is also not missed
     * @param time time in ms to wait
     */
    public void safeWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            autonomousController.runAutoControl();
        }
    }

    //TODO: Arm function
    /*public void moveMajorArmToPickupAndOpenClaw(){
        autonomousController.moveAutoMajorArmPickup();
        safeWait(1000);
        autonomousController.openAutoMajorClaw();
    }*/

    //TODO: Arm function
    /*public void moveMajorArmToParkingAfterClosingClaw(){
        autonomousController.closeAutoMajorClaw();
        safeWait(1000);
        autonomousController.moveAutoMajorArmPark();
        safeWait(1000);
    }*/

    public void rotateCarousal() {
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            autonomousController.autoSSSpinnerState = AutonomousController.AUTO_SSSPINNER_STATE.CLOCKWISE;
        } else {
            autonomousController.autoSSSpinnerState = AutonomousController.AUTO_SSSPINNER_STATE.ANTICLOCKWISE;
        }
        autonomousController.runAutoControl();
        safeWait(4000);
        autonomousController.autoSSSpinnerState = AutonomousController.AUTO_SSSPINNER_STATE.STOPPED;
        autonomousController.runAutoControl();
    }

    //Drops pre-loaded box at the correct level determined by capstone position
    //TODO: Update code to use autoController instead of accessing the subsystems directly
    public void dropBoxToLevel(){
        switch(targetZone) {
            case LEVEL1:
                //If Capstone on Level 1, Drops Pre-Loaded Box on Level 1
                dropFreightLevel1();
                break;
            case LEVEL2:
                //If Capstone on Level 1, Drops Pre-Loaded Box on Level 2
                autonomousController.autoSSElevatorSetToLevel2();
                safeWait(1000);
                autonomousController.autoBucketSetToDrop();
                safeWait(1000);
                autonomousController.autoBucketSetToCollect();
                safeWait(1000);
                break;
            case LEVEL3:
                //If Capstone on Level 1, Drops Pre-Loaded Box on Level 3
                autonomousController.autoSSElevatorSetToLevel3();
                safeWait(1000);
                autonomousController.autoBucketSetToDrop();
                safeWait(1000);
                autonomousController.autoBucketSetToCollect();
                safeWait(1000);
                break;
        }

    }

    public void dropFreightLevel1(){
        autonomousController.autoSSElevatorSetToLevel1();
        safeWait(1000);
        autonomousController.autoBucketSetToDrop();
        safeWait(1000);
        autonomousController.autoBucketSetToCollect();
        safeWait(1000);
    }

    public void moveElevatorToLevel1(){
        autonomousController.autoSSElevatorSetToLevel1();
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_FLAG is : ", DEBUG_FLAG);

        telemetry.addData("GameField.playingAlliance : ", GameField.playingAlliance);
        telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
        telemetry.addData("GameField.currentPose : ", GameField.currentPose);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", driveTrain.driveMode);
        telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
        telemetry.addData("Battery Power", driveTrain.getBatteryVoltage(hardwareMap));

        //****** Vision Debug *****
        telemetry.addData("Target Label : ", vision.detectedLabel);
        telemetry.addData("Target Left : ", vision.detectedLabelLeft);
        telemetry.addData("Target Right : ", vision.detectedLabelRight);
        telemetry.addData("Target Top : ", vision.detectedLabelTop);
        telemetry.addData("Target Bottom : ", vision.detectedLabelBottom);

        telemetry.addData("Intake State : ", ssIntake.getSSIntakeMotorState());
        telemetry.addData("Elevator level value : ", ssElevator.getElevatorPosition());
        telemetry.addData("Elevator Encoder Current :", ssElevator.elevatorMotor.getCurrentPosition());
        telemetry.addData("Elevator Encoder Target:", ssElevator.elevatorMotor.getTargetPosition());
        telemetry.addData("Bucket State : ", ssBucket.getBucketServoState());
        telemetry.addData("Bucket State : ", ssBucket.bucketServo.getPosition());
        telemetry.addData("SSSpinner State : ", ssSpinner.getSSSpinnerMotorState() );

        telemetry.update();

    }
}

