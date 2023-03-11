package org.firstinspires.ftc.teamcode.GameOpModes;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.SSArm;
import org.firstinspires.ftc.teamcode.Subsystems.SSClaw;
import org.firstinspires.ftc.teamcode.Subsystems.SSElevator;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * Adapted from FTC WIRES Autonomous Example for dropping preloaded cone, picking and dropping 2 cones and park
 */
@Autonomous(name = "Autonomous Plus One", group = "00-Autonomous", preselectTeleOp = "SSTeleOp")
public class AutoOpModePlusOne extends LinearOpMode {

    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    public static START_POSITION startPosition;

    public Vision vision;
    public DriveTrain driveTrain;
    public SSClaw claw;
    public SSArm arm;
    public SSElevator elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        /*Create your Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        claw = new SSClaw(hardwareMap);
        arm = new SSArm(hardwareMap);
        elevator = new SSElevator(hardwareMap, this);

        vision = new Vision(hardwareMap);

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();

        // Initiate Camera on Init.
        vision.activateVuforiaTensorFlow();

        //Build Autonomous trajectory to be used based on starting position selected
        buildAuto();
        driveTrain.getLocalizer().setPoseEstimate(initPose);

        while (!isStopRequested() && !opModeIsActive()) {
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            vision.runVuforiaTensorFlow();
            telemetry.clearAll();
            telemetry.addData("Start FTC Wires (ftcwires.org) Autonomous Mode adopted for Team", "TEAM NUMBER");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Vision identified Parking Location", vision.identifiedparkingLocation);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            // close the claw to grip the pre-loaded cone
            claw.setGripClose();

            //Stop Vision process
            vision.deactivateVuforiaTensorFlow();

            //Build parking trajectory based on last detected target by vision
            buildParking();

            //run Autonomous trajectory
            runAutoAndParking();
        }

        //Trajectory is completed, display Parking complete
        parkingComplete();
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryParking,
            trajectoryDropPreloadedCone,
            trajectoryPushCone,
            trajectoryPreloadedToAlignConeStack,
            trajectoryGoToConeStack,
            trajectoryConeStackToDrop1Plus,
            trajectoryGoToConeStack1Plus,trajectoryGoToDropCone1Plus;


    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d alignToStackPose;
    Pose2d pickConePose;
    Pose2d dropCone0Pose;
    Pose2d dropPlus1Pose;
    Pose2d pushConePose;
    Pose2d parkPose;
    Pose2d dropCone0PoseAngle;
    Pose2d dropPlus1PoseAngle;
    double pushTurnAngle, straightenAngle;
    double pushCone, comeBack;

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        switch (startPosition) {
            /*
            case BLUE_LEFT:
                initPose = new Pose2d(-54, 36, Math.toRadians(0));//Starting pose
                dropCone0Pose = new Pose2d(-27, 37, Math.toRadians(0));
                dropCone0PoseAngle = new Pose2d(-24, 34, Math.toRadians(-60));
                pushConePose= new Pose2d(-1, 36, Math.toRadians(0));
                alignToStackPose = new Pose2d(0, 36, Math.toRadians(90));
                pickConePose = new Pose2d(-4, 54, Math.toRadians(90));
                dropPlus1Pose = new Pose2d(-4, 30, Math.toRadians(90));
                dropPlus1PoseAngle = new Pose2d(4.5, 18.5, Math.toRadians(135));
                pushTurnAngle = -30;
                straightenAngle = 90;
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-54, -36, Math.toRadians(0));//Starting pose
                dropCone0Pose = new Pose2d(-27, -37, Math.toRadians(0));
                dropCone0PoseAngle = new Pose2d(-24, -34, Math.toRadians(60));
                pushConePose= new Pose2d(-1, -36, Math.toRadians(0));
                alignToStackPose = new Pose2d(0, -36, Math.toRadians(90));
                pickConePose = new Pose2d(0, -53, Math.toRadians(90));
                dropPlus1Pose = new Pose2d(3, -18, Math.toRadians(90));
                dropPlus1PoseAngle = new Pose2d(4.5, -18.5, Math.toRadians(45));
                pushTurnAngle = 30;
                straightenAngle = -90;
                break;
            */
            case BLUE_LEFT:
            case RED_LEFT:
                initPose = new Pose2d(54, -36, Math.toRadians(180));//Starting pose
                dropCone0Pose = new Pose2d(21, -37, Math.toRadians(180));
                dropCone0PoseAngle = new Pose2d(20, -37, Math.toRadians(180-60));
                pushConePose= new Pose2d(1, -36, Math.toRadians(180));
                alignToStackPose = new Pose2d(3, -36, Math.toRadians(270));
                pickConePose = new Pose2d(4.5, -54, Math.toRadians(270));
                dropPlus1Pose = new Pose2d(4, -17, Math.toRadians(270));
                dropPlus1PoseAngle = new Pose2d(5, -19, Math.toRadians(315));
                pushTurnAngle = -30;
                straightenAngle = 90;
                pushCone = 4;
                comeBack = 4;
                break;
            case BLUE_RIGHT:
            case RED_RIGHT:
                initPose = new Pose2d(54, 36, Math.toRadians(180));//Starting pose
                dropCone0Pose = new Pose2d(27, 37, Math.toRadians(180));
                dropCone0PoseAngle = new Pose2d(21, 36, Math.toRadians(180+60));
                pushConePose= new Pose2d(1, 36, Math.toRadians(180));
                alignToStackPose = new Pose2d(4, 36, Math.toRadians(90));
                pickConePose = new Pose2d(6, 54.5, Math.toRadians(90));
                dropPlus1Pose = new Pose2d(3.5, 17, Math.toRadians(90));
                dropPlus1PoseAngle = new Pose2d(5.5, 19, Math.toRadians(45));
                pushTurnAngle = 30;
                straightenAngle = -90;
                pushCone = 1;
                comeBack = 1;
                break;
        }

        // Drop preloaded cone trajectory
        trajectoryDropPreloadedCone = driveTrain.trajectorySequenceBuilder(initPose)
                //Uncomment following line to slow down turn if needed.
                .setVelConstraint(getVelocityConstraint(
                        0.75 * DriveConstants.MAX_VEL/*Slower velocity*/,
                        0.5 * DriveConstants.MAX_ANG_VEL, /*Slower angular velocity*/
                        DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(dropCone0Pose)
                .lineToLinearHeading(dropCone0PoseAngle)
                .build();

/*        trajectoryPushCone = driveTrain.trajectorySequenceBuilder(dropCone0Pose)
                //Uncomment following line to slow down turn if needed.
                .setVelConstraint(getVelocityConstraint(
                        0.5 * DriveConstants.MAX_VEL*//*Slower velocity*//*,
                        0.5 * DriveConstants.MAX_ANG_VEL, *//*Slower angular velocity*//*
                        DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(pushConePose)
                .build();*/

        trajectoryPreloadedToAlignConeStack = driveTrain.trajectorySequenceBuilder(dropCone0PoseAngle)
                //Uncomment following line to slow down turn if needed.
                .setVelConstraint(getVelocityConstraint(
                        0.75 * DriveConstants.MAX_VEL/*Slower velocity*/,
                        0.5 * DriveConstants.MAX_ANG_VEL, /*Slower angular velocity*/
                        DriveConstants.TRACK_WIDTH))
                .turn(Math.toRadians(pushTurnAngle))
                .forward(pushCone)
                .back(comeBack)
                .turn(Math.toRadians(straightenAngle))
                //.lineToLinearHeading(dropCone0Pose)
                .lineToLinearHeading(alignToStackPose)
                .build();

        trajectoryGoToConeStack = driveTrain.trajectorySequenceBuilder(alignToStackPose)
                //Uncomment following line to slow down turn if needed.
                .setVelConstraint(getVelocityConstraint(
                        0.75 * DriveConstants.MAX_VEL/*Slower velocity*/,
                        0.5 * DriveConstants.MAX_ANG_VEL, /*Slower angular velocity*/
                        DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(pickConePose)
                .build();

        trajectoryGoToDropCone1Plus = driveTrain.trajectorySequenceBuilder(dropPlus1PoseAngle)
                //Uncomment following line to slow down turn if needed.
                .setVelConstraint(getVelocityConstraint(
                        0.75 * DriveConstants.MAX_VEL/*Slower velocity*/,
                        0.5 * DriveConstants.MAX_ANG_VEL, /*Slower angular velocity*/
                        DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(dropPlus1Pose)
                .build();

        trajectoryGoToConeStack1Plus = driveTrain.trajectorySequenceBuilder(dropPlus1Pose)
                //Uncomment following line to slow down turn if needed.
                .setVelConstraint(getVelocityConstraint(
                        0.75 * DriveConstants.MAX_VEL/*Slower velocity*/,
                        0.5 * DriveConstants.MAX_ANG_VEL, /*Slower angular velocity*/
                        DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(pickConePose)
                .build();

        trajectoryConeStackToDrop1Plus = driveTrain.trajectorySequenceBuilder(pickConePose)
                //Uncomment following line to slow down turn if needed.
                .setVelConstraint(getVelocityConstraint(
                        0.75 * DriveConstants.MAX_VEL/*Slower velocity*/,
                        0.5 * DriveConstants.MAX_ANG_VEL, /*Slower angular velocity*/
                        DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(dropPlus1Pose)
                .lineToLinearHeading(dropPlus1PoseAngle)
                .build();

    }

    //Build parking trajectory based on target detected by vision
    public void buildParking() {
        switch (startPosition) {
            /*case BLUE_LEFT:
                switch (vision.identifiedparkingLocation) {
                    case 1:
                        parkPose = new Pose2d(-27, 64, Math.toRadians(270));
                        break; // Location 1
                    case 2:
                        parkPose = new Pose2d(-27, 36.5, Math.toRadians(270));
                        break; // Location 2
                    case 3:
                        parkPose = new Pose2d(-27, 8, Math.toRadians(270));
                        break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                switch (vision.identifiedparkingLocation) {
                    case 1:
                        parkPose = new Pose2d(-27, -8, Math.toRadians(90));
                        break; // Location 1
                    case 2:
                        parkPose = new Pose2d(-27, -36.5, Math.toRadians(90));
                        break; // Location 2
                    case 3:
                        parkPose = new Pose2d(-27, -64, Math.toRadians(90));
                        break; // Location 3
                }
                break;*/
            case BLUE_LEFT:
            case RED_LEFT:
                switch (vision.identifiedparkingLocation) {
                    case 1:
                        parkPose = new Pose2d(0, -54, Math.toRadians(360));
                        break; // Location 1
                    case 2:
                        parkPose = new Pose2d(0, -36.5, Math.toRadians(360));
                        break; // Location 2
                    case 3:
                        parkPose = new Pose2d(0, -8, Math.toRadians(360));
                        break; // Location 3
                }
                break;
            case BLUE_RIGHT:
            case RED_RIGHT:
                switch (vision.identifiedparkingLocation) {
                    case 1:
                        parkPose = new Pose2d(1, 12, Math.toRadians(360));
                        break; // Location 1
                    case 2:
                        parkPose = new Pose2d(0, 36.5, Math.toRadians(360));
                        break; // Location 2
                    case 3:
                        parkPose = new Pose2d(0, 56, Math.toRadians(360));
                        break; // Location 3
                }
                break;
        }

        trajectoryParking = driveTrain.trajectorySequenceBuilder(dropPlus1Pose)
                .lineToLinearHeading(parkPose)
                .forward(4)
                .build();
    }

    //Run Auto trajectory and parking trajectory
    public void runAutoAndParking() {
        telemetry.setAutoClear(false);
        telemetry.addData("Running FTC Wires (ftcwires.org) Autonomous Mode adopted for Team:", "TEAM NUMBER");
        telemetry.addData("---------------------------------------", "");
        telemetry.update();
/*
    Close claw
    lift arm to mid junction (init pose)
    go to drop 0- 45 degrees (drop 0 pose)
    drop preloaded cone 1 (open claw)
    turn to midway pose 0 angle (midway pose)
    Align to stack pose- turn 90 degrees (align to stack 90 pose)
    Move arm to pick cone # 5, highest cone position (move arm pose)
    Move to cone stack (pickConePose)
    Close Claw and raise elevator
    go to drop +1 position (drop 1+ pose)
    turn 45 degrees (drop 1+ 45 degree pose)
    drop cone 1- open claw
    align to stack pose (align to stack pose- 45 degrees)
    Move arm to pick cone # 4, highest cone position
    Move to cone stack
    Close Claw and raise elevator
    go to drop +1 position
    turn 45 degrees
    align to stack pose
    drop cone 2- open claw
    park
 */
        raiseArmAndElevator();

        // Drop pre loaded cone
        driveTrain.followTrajectorySequence(trajectoryDropPreloadedCone);
   //     driveTrain.turn(Math.toRadians(dropCone0Pose.getHeading() - 65));
        dropCone(0);
        // straighten out and push
     //   driveTrain.turn(Math.toRadians(dropCone0Pose.getHeading() + 65));
        //driveTrain.followTrajectorySequence(trajectoryPushCone);
        //driveTrain.followTrajectorySequence(trajectoryDropPreloadedCone);

        // Align to cone stack
        driveTrain.followTrajectorySequence(trajectoryPreloadedToAlignConeStack);

        // Pick up a cone
        arm.moveArmToConeIntakePosition(1);
        driveTrain.followTrajectorySequence(trajectoryGoToConeStack);
        pickCone(1);
        raiseArmAndElevator();

        // Drop the cone
        driveTrain.followTrajectorySequence(trajectoryConeStackToDrop1Plus);
        //driveTrain.turn(Math.toRadians(dropPlus1Pose.getHeading() + 60));
        dropCone(1);
        //driveTrain.turn(Math.toRadians(dropPlus1Pose.getHeading() - 60));
        driveTrain.followTrajectorySequence(trajectoryGoToDropCone1Plus);
        // Pick up another cone
        arm.moveArmToConeIntakePosition(2);
        driveTrain.followTrajectorySequence(trajectoryGoToConeStack1Plus);
        pickCone(2);
        raiseArmAndElevator();

        // Drop the cone
        driveTrain.followTrajectorySequence(trajectoryConeStackToDrop1Plus);
        //driveTrain.turn(Math.toRadians(dropPlus1Pose.getHeading() + 60));
        dropCone(2);
        //driveTrain.turn(Math.toRadians(dropPlus1Pose.getHeading() - 60));
        driveTrain.followTrajectorySequence(trajectoryGoToDropCone1Plus);

        // Park
        driveTrain.followTrajectorySequence(trajectoryParking);

        // Lower the arm
        lowerArm();
    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(int coneCount) {
        claw.setGripClose();
        safeWait(1000);
        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(int coneCount) {


        claw.setGripOpen();
        safeWait(1500);
        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
    }

    public void parkingComplete() {
        telemetry.addData("Parked in Location", vision.identifiedparkingLocation);
        telemetry.update();
    }

    public void safeWait(double time) {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            //wait
        }
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while (!isStopRequested()) {
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous Mode adopted for Team:", "TEAM NUMBER");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:", "");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if (gamepad1.x) {
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if (gamepad1.y) {
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if (gamepad1.b) {
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if (gamepad1.a) {
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    public void raiseArmAndElevator() {
        // Move arm to mid
        arm.moveArmMid();
        arm.runArmToLevel(arm.motorPowerToRun);
    }

    public void lowerArm() {
        // Move arm low
        arm.moveArmIntakeForward();
        arm.runArmToLevel(arm.motorPowerToRun);
    }
}


// trajectoryPush = driveTrain.trajectorySequenceBuilder(initPose)
//.lineToLinearHeading(pushPose)
//Uncomment following line to slow down turn if needed.
//.setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
//.setVelConstraint(getVelocityConstraint(
// 0.5 * DriveConstants.MAX_VEL/*Slower velocity*/,
// 0.5 * DriveConstants.MAX_ANG_VEL, /*Slower angular velocity*/
// DriveConstants.TRACK_WIDTH))
//.lineToLinearHeading(pushPose)
// .build();


// trajectoryAuto =driveTrain.trajectorySequenceBuilder(pushPose)
//  .lineToLinearHeading(midWayPose)
//Uncomment following line to slow down turn if needed.
//.setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
//  .setVelConstraint(getVelocityConstraint(
//   0.5*DriveConstants.MAX_VEL/*Slower velocity*/,
//  0.5*DriveConstants.MAX_ANG_VEL, /*Slower angular velocity*/
// DriveConstants.TRACK_WIDTH))
//.lineToLinearHeading(dropConePose0)
// .build();

