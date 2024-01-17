/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSBucket;
import org.firstinspires.ftc.teamcode.SubSystems.SSClimber;
import org.firstinspires.ftc.teamcode.SubSystems.SSElevator;
import org.firstinspires.ftc.teamcode.SubSystems.SSIndicators;
import org.firstinspires.ftc.teamcode.SubSystems.SSIntake;
import org.firstinspires.ftc.teamcode.SubSystems.SSRocketLauncher;
import org.firstinspires.ftc.teamcode.SubSystems.VisionTfod;

/**
 * Superstellar Autonomous
 */
@Autonomous(name = "SS Autonomous Mode Drop-Park", group = "00-Autonomous", preselectTeleOp = "SS TeleOp")
public class SSAutonomousModeRev1 extends LinearOpMode {

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public SSIntake ssIntake;
    public SSElevator ssElevator;
    public SSBucket ssBucket;
    public SSRocketLauncher ssRocketLauncher;
    public SSIndicators ssIndicators;
    public SSClimber ssClimber;
    public VisionTfod visionTfodFront;

    //Static Class for knowing system state

    public Pose2d startPose = GameField.ORIGINPOSE;

    public double timeToWait = 0.0;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);
    public ElapsedTime startTimer = new ElapsedTime(MILLISECONDS);

    public String visionModelAssetFile;

    @Override
    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.SUPERSTELLAR_AUTONOMOUS;

        /* Set Initial State of any subsystem when OpMode is to be started*/
        initSubsystems();

        configureWaitTime();

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", GameField.startPosition);

        // Initiate Camera on Init.
        visionTfodFront.initTfod(visionModelAssetFile);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", GameField.startPosition);

            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            visionTfodFront.runTfodTensorFlow();
            telemetry.addData("Vision identified Parking Location", visionTfodFront.identifiedSpikeMarkLocation);
            telemetry.update();
        }

        //Stop Vision process
        /*if (visionPortal.getCameraState() != CAMERA_DEVICE_CLOSED) {
            //visionPortal.stopStreaming();
            visionPortal.close();
        }*/

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            gameTimer.reset();
            startTimer.reset();

            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose1a = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0, 0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(15,0,0);

        switch (GameField.startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(0, 0, Math.toRadians(0));
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(visionTfodFront.identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(24, -1, Math.toRadians(47));
                        dropYellowPixelPose = new Pose2d(17.5, 37.5, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(22, -2, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(24, 37,  Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(23, -1, Math.toRadians(-56));
                        dropYellowPixelPose = new Pose2d(32, 38, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(2, 37, Math.toRadians(-90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(8, 30, Math.toRadians(-90));
                break;

            case RED_LEFT:
                initPose = new Pose2d(0, 0, Math.toRadians(0));
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(visionTfodFront.identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(-2, 18, Math.toRadians(129));
                        dropYellowPixelPose = new Pose2d(48, 48, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(-6, 12, Math.toRadians(172));
                        dropYellowPixelPose = new Pose2d(30, -38,  Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(-4, 19, Math.toRadians(133));
                        dropYellowPixelPose = new Pose2d(24, 48, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(14, -13, Math.toRadians(45));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(8, -30, Math.toRadians(90));
                break;

            case BLUE_RIGHT: //GOOD
                initPose = new Pose2d(0, 0, Math.toRadians(0));
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(visionTfodFront.identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(0,  11, Math.toRadians(185));
                        dropYellowPixelPose = new Pose2d(107, -42, Math.toRadians(80));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(-3, 12, Math.toRadians(113));
                        dropYellowPixelPose = new Pose2d(99, -40, Math.toRadians(84));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(4, 6, Math.toRadians(80));
                        dropYellowPixelPose = new Pose2d(91, -39, Math.toRadians(86));
                        break;
                }
                midwayPose1 = new Pose2d(8, -8, Math.toRadians(0));
                midwayPose1a = new Pose2d(18, -18, Math.toRadians(-90));
                intakeStack = new Pose2d(52, -19,Math.toRadians(-90));
                midwayPose2 = new Pose2d(52, 62, Math.toRadians(-90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(50, 84, Math.toRadians(-90));
                break;

            case RED_RIGHT:
                initPose = new Pose2d(0, 0, Math.toRadians(0));
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(visionTfodFront.identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(25, 5, Math.toRadians(51));
                        dropYellowPixelPose = new Pose2d(33, -46, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(24, -2, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(27, -46, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(21, 0, Math.toRadians(-47));
                        dropYellowPixelPose = new Pose2d(30, -46, Math.toRadians(93));
                        break;
                }
                midwayPose1 = new Pose2d(2, -27, Math.toRadians(90));
                midwayPose1a = new Pose2d(18, 18, Math.toRadians(90));
                intakeStack = new Pose2d(52, 19,Math.toRadians(90));
                midwayPose2 = new Pose2d(52, -62, Math.toRadians(90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(4, -30, Math.toRadians(85));
                break;
        }

        drive.pose = initPose;


        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        //.strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .splineToLinearHeading(dropPurplePixelPose,0)
                        .build());

        //TODO : Code to drop Purple Pixel on Spike Mark
        safeWaitSeconds(1);
        ssIntake.runIntakeReverse(300);

        safeWaitSeconds(timeToWait);
        //Move robot to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        //For Blue Right and Red Left, intake pixel from stack
        if (GameField.startPosition == GameField.START_POSITION.BLUE_RIGHT ||
                GameField.startPosition == GameField.START_POSITION.RED_LEFT) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)
                            .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                            .build());

            //TODO : Code to intake pixel from stack
            safeWaitSeconds(1);

            //Move robot to midwayPose2 and to dropYellowPixelPose
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .build());
        }

        safeWaitSeconds(waitSecondsBeforeDrop);

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(dropYellowPixelPose,0)
                        .build());


        //TODO : Code to drop Pixel on Backdrop
        safeWaitSeconds(1);
        ssElevator.moveElevatorLevelHigh();
        safeWaitSeconds(2);
        ssBucket.setBucketDropPosition();
        safeWaitSeconds(2);
        ssBucket.setBucketCarryPosition();
        safeWaitSeconds(1);
        ssElevator.moveElevatorPickPosition();
        safeWaitSeconds(2);

        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());
        ssIntake.openIntakeLatch();
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addLine("Initializing Superstellar Autonomous Mode:");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X / ▢)");
            telemetry.addData("    Blue Right ", "(Y / Δ)");
            telemetry.addData("    Red Left    ", "(B / O)");
            telemetry.addData("    Red Right  ", "(A / X)");
            if(gamepad1.x){
                GameField.startPosition = GameField.START_POSITION.BLUE_LEFT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                visionModelAssetFile = VisionTfod.TFOD_MODEL_ASSET_BLUE;
                break;
            }
            if(gamepad1.y){
                GameField.startPosition = GameField.START_POSITION.BLUE_RIGHT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                visionModelAssetFile = VisionTfod.TFOD_MODEL_ASSET_BLUE;
                break;
            }
            if(gamepad1.b){
                GameField.startPosition = GameField.START_POSITION.RED_LEFT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                visionModelAssetFile = VisionTfod.TFOD_MODEL_ASSET_RED;
                break;
            }
            if(gamepad1.a){
                GameField.startPosition = GameField.START_POSITION.RED_RIGHT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                visionModelAssetFile = VisionTfod.TFOD_MODEL_ASSET_RED;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    public void initSubsystems(){

        telemetry.setAutoClear(false);

        //Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap, new Pose2d(0,0,0), telemetry);
        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        telemetry.addData("DriveTrain Initialized with Pose:",driveTrain.toStringPose2d(driveTrain.pose));
        telemetry.update();

        /* Create ssIntake */
        ssIntake = new SSIntake(hardwareMap, telemetry);
        telemetry.addLine("ssIntake Initialized");
        telemetry.update();

        /* Create ssElevator */
        ssElevator = new SSElevator(hardwareMap, telemetry);
        telemetry.addLine("ssElevator Initialized");
        telemetry.update();

        /* Create ssHoldingPen */
        telemetry.addLine("ssHoldingPen Initialized");
        telemetry.update();

        /* Create ssClaw */
        ssBucket = new SSBucket(hardwareMap, telemetry);
        telemetry.addLine("ssClaw Initialized");
        telemetry.update();

        /* Create ssRocketLauncher */
        ssRocketLauncher = new SSRocketLauncher(hardwareMap, telemetry);
        telemetry.addLine("ssRocketLauncher Initialized");
        telemetry.update();

        /* Create ssClimber */
        ssClimber = new SSClimber(hardwareMap, telemetry);
        telemetry.addLine("ssClimber Initialized");
        telemetry.update();

        /* Create ssIndicators */
        ssIndicators = new SSIndicators(hardwareMap, telemetry);
        telemetry.addLine("ssIndicators Initialized");
        telemetry.update();


        /* Create Vision */
        visionTfodFront = new VisionTfod(hardwareMap, telemetry, "Webcam 1");
        telemetry.addLine("VisionTfod Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain,
                ssIntake, ssElevator, ssBucket, ssIndicators, ssRocketLauncher, ssClimber, telemetry, this);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /* Get last position after Autonomous mode ended from static class set in Autonomous */
        if ( GameField.poseSetInAutonomous) {
            driveTrain.pose = GameField.currentPose;
            //driveTrain.getLocalizer().setPoseEstimate(GameField.currentPose);
        } else {
            driveTrain.pose = startPose;
            //driveTrain.getLocalizer().setPoseEstimate(startPose);
        }

        //GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;

        telemetry.addLine("+++++++++++++++++++++++");
        telemetry.addLine("Init Completed, All systems Go! Let countdown begin. Waiting for Start");
        telemetry.update();
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_LEVEL is : ", GameField.debugLevel);
        telemetry.addData("Robot ready to start","");

        if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
            telemetry.addLine("Running Superstellar Autonomous Mode");
            telemetry.addData("Game Timer : ", gameTimer.time());
            //telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            //telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            //telemetry.addData("startPose : ", startPose);

            driveTrain.printDebugMessages();
            ssIndicators.printDebugMessages();
        }
        telemetry.update();
    }

    public void configureWaitTime() {

        while (!isStopRequested()) {

            telemetry.clearAll();
            telemetry.addData("Configure wait time (in seconds) using DPad on gamepad 1:", "");
            telemetry.addData("Press DPad UP to increase", "");
            telemetry.addData("Press DPad DOWN to decrease", "");
            telemetry.addData("Press DPad LEFT to exit", "(X / ▢)");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Wait Time: ", timeToWait);

            telemetry.update();

            if (gamepad1.dpad_up) {
                timeToWait = timeToWait + 2.0;
                if (timeToWait > 15.0) {
                    timeToWait = 15.0;
                }
            }

            if (gamepad1.dpad_down) {
                timeToWait = timeToWait - 2.0;
                if (timeToWait < 0.0) {
                    timeToWait = 0.0;
                }
            }

            if (gamepad1.dpad_left) {
                break;
            }

            // Prevents accidental button presses or rapid fire button presses
            // (debounce)
            safeWaitSeconds(0.2);
        }
    }


}   // end class
