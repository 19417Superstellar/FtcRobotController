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
 * FTC WIRES Autonomous Example for dropping preloaded cone, picking and dropping 2 cones and park
 */
@Autonomous(name = "FTC Wires Autonomous", group = "00-Autonomous", preselectTeleOp = "SSTeleOp")
public class AutoOpMode extends LinearOpMode {

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

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryAuto, trajectoryParking, trajectoryPush;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d pushPose;
    Pose2d midWayPose;
    Pose2d dropConePose0;
    Pose2d parkPose;


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
        buildPush();
        buildAuto();

        driveTrain.getLocalizer().setPoseEstimate(initPose);

        while (!isStopRequested() && !opModeIsActive()) {
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            vision.runVuforiaTensorFlow();
            telemetry.clearAll();
            telemetry.addData("Start FTC Wires (ftcwires.org) Autonomous Mode adopted for Team", "19417");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Vision identified Parking Location", vision.identifiedparkingLocation);
            telemetry.addData("Vision identified label", vision.detectedLabel);
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

    //Run Auto trajectory and parking trajectory
    public void runAutoAndParking() {
        telemetry.setAutoClear(false);
        telemetry.addData("Running FTC Wires (ftcwires.org) Autonomous Mode adopted for Team:", "19417");
        telemetry.addData("---------------------------------------", "");
        telemetry.update();

        //Raise the arm before we start moving
        raiseArmAndElevator();
        safeWait(2000);

        // Push the cone
        driveTrain.followTrajectorySequence(trajectoryPush);
        safeWait(1000);

        driveTrain.followTrajectorySequence(trajectoryAuto);

        // Drop the cone
        safeWait(1000);
        dropCone(0); //Drop preloaded Cone
//        safeWait(2000);

        // Go park
        driveTrain.followTrajectorySequence(trajectoryParking);
        safeWait(2000);

        // Lower the arm
        lowerArm();

    }

    public void buildPush() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-54, 36, Math.toRadians(0)); //Starting pose
                pushPose = new Pose2d(-25, 36, Math.toRadians(0));
            case BLUE_RIGHT:
                initPose = new Pose2d(-54, -36, Math.toRadians(0));//Starting pose
                pushPose = new Pose2d(-25, -36, Math.toRadians(0));
            case RED_LEFT:
                initPose = new Pose2d(54, -36, Math.toRadians(180));//Starting pose
                pushPose = new Pose2d(17, -36, Math.toRadians(180));
            case RED_RIGHT:
                initPose = new Pose2d(54, 36, Math.toRadians(180)); //Starting pose
                pushPose = new Pose2d(25, 36, Math.toRadians(180));
        }

        trajectoryPush = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(pushPose)
                //Uncomment following line to slow down turn if needed.
                //.setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .setVelConstraint(getVelocityConstraint(
                        0.5 * DriveConstants.MAX_VEL/*Slower velocity*/,
                        0.5 * DriveConstants.MAX_ANG_VEL, /*Slower angular velocity*/
                        DriveConstants.TRACK_WIDTH))
                // .lineToLinearHeading(dropConePose0)
                .build();
    }

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                midWayPose = new Pose2d(-29, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                dropConePose0 = new Pose2d(-29, 20, Math.toRadians(0)); //Choose the pose to move to the stack of cones
                break;
            case BLUE_RIGHT:
                midWayPose = new Pose2d(-28, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                dropConePose0 = new Pose2d(-28, -22, Math.toRadians(0)); //Choose the pose to move to the stack of cones
                break;
            case RED_LEFT:
                midWayPose = new Pose2d(29, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                dropConePose0 = new Pose2d(28.5, -19, Math.toRadians(180)); //Choose the pose to move to the stack of cones
                break;
            case RED_RIGHT:
                midWayPose = new Pose2d(28, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                dropConePose0 = new Pose2d(28, 21, Math.toRadians(180)); //Choose the pose to move to the stack of cones
                break;
        }

        trajectoryAuto =driveTrain.trajectorySequenceBuilder(pushPose)
                .lineToLinearHeading(midWayPose)
                //Uncomment following line to slow down turn if needed.
                //.setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .setVelConstraint(getVelocityConstraint(
                        0.5*DriveConstants.MAX_VEL/*Slower velocity*/,
                        0.5*DriveConstants.MAX_ANG_VEL, /*Slower angular velocity*/
                        DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(dropConePose0)
                .build();

    }


    //Build parking trajectory based on target detected by vision
    public void buildParking() {
        switch (startPosition) {
            case BLUE_LEFT:
                switch (vision.identifiedparkingLocation) {
                    case 1:
                        parkPose = new Pose2d(-27, 64, Math.toRadians(0));
                        break; // Location 1
                    case 2:
                        parkPose = new Pose2d(-27, 36.5, Math.toRadians(0));
                        break; // Location 2
                    case 3:
                        parkPose = new Pose2d(-27, 8, Math.toRadians(0));
                        break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                switch (vision.identifiedparkingLocation) {
                    case 1:
                        parkPose = new Pose2d(-27, -8, Math.toRadians(0));
                        break; // Location 1
                    case 2:
                        parkPose = new Pose2d(-27, -36.5, Math.toRadians(0));
                        break; // Location 2
                    case 3:
                        parkPose = new Pose2d(-27, -64, Math.toRadians(0));
                        break; // Location 3
                }
                break;
            case RED_LEFT:
                switch (vision.identifiedparkingLocation) {
                    case 1:
                        parkPose = new Pose2d(27, -64, Math.toRadians(180));
                        break; // Location 1
                    case 2:
                        parkPose = new Pose2d(27, -36.5, Math.toRadians(180));
                        break; // Location 2
                    case 3:
                        parkPose = new Pose2d(27, -8, Math.toRadians(180));
                        break; // Location 3
                }
                break;
            case RED_RIGHT:
                switch (vision.identifiedparkingLocation) {
                    case 1:
                        parkPose = new Pose2d(27, 8, Math.toRadians(180));
                        break; // Location 1
                    case 2:
                        parkPose = new Pose2d(27, 36.5, Math.toRadians(180));
                        break; // Location 2
                    case 3:
                        parkPose = new Pose2d(27, 64, Math.toRadians(180));
                        break; // Location 3
                }
                break;
        }

        trajectoryParking = driveTrain.trajectorySequenceBuilder(dropConePose0)
                .lineToLinearHeading(parkPose)
                .build();
    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(int coneCount) {
        /*TODO: Add code to pick Cone 1 from stack*/

        // Open claw
        claw.setGripOpen();

        // Move the arm to intake position based on coneCount
        arm.moveArmToConeIntakePosition(coneCount);
        arm.runArmToLevel(arm.motorPowerToRun);

        // Close the claw
        claw.setGripClose();

        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(int coneCount) {
        // Open claw
        //arm.moveSSArmSlightlyDown();
        //arm.moveSSArmSlightlyDown();

        safeWait(1000);

        claw.setGripOpen();

        //arm.moveSSArmSlightlyUp();
        //arm.moveSSArmSlightlyUp();


        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
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
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous Mode adopted for Team:", "19417");
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
}
