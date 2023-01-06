package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Controllers.AutonomousController;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Subsystems.SSClaw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.SSArm;
import org.firstinspires.ftc.teamcode.Subsystems.SSElevator;
import org.firstinspires.ftc.teamcode.Subsystems.SSClaw;
import org.firstinspires.ftc.teamcode.Subsystems.SSVision;

public class Autonomous_RR_Forward {


    /**
     * Path and actions for autonomous mode starting from Red Warehouse position
     * 1.  Robot starts position with bucket in transport state in Level 1 with pre-loaded box in it.
     * 2.  Call Vision function to detect duck/Team Marker position and set level to drop pre-loaded box
     * 3.  Call roadrunner function to move to duck/Team Marker position
     * 4.  Call arm function to pick up duck / Team Marker
     * 5.  Call roadrunner function move to Alliance Shipping Hub
     * 6.  Call elevator function to raise to correct level drop preloaded box
     * 7.  Call bucket function to drop preloaded box
     * 8.  Call bucket function to change to transport state
     * 9.  Call elevator function to move to Level 1 position
     * 10. Call roadrunner function to move to parking position in storage area through gap
     */
    @Autonomous(name = "Autonomous_Red_Warehouse", group = "00-Autonomous", preselectTeleOp = "SSTeleOp")
    public class Autonomous_Red_Warehouse extends LinearOpMode {

        public boolean DEBUG_FLAG = true;

        public GamepadController gamepadController;
        public AutonomousController autonomousController;
        public DriveTrain driveTrain;
        public SSElevator SSElevator;
        public SSArm SSArm;
        public SSClaw SSClaw;


        public SSVision vision;
        public final Pose2d RED_WAREHOUSE_STARTPOS = new Pose2d(61, 7, Math.toRadians(0));
        public Pose2d startPose = RED_WAREHOUSE_STARTPOS;


        boolean parked = false;
        boolean autonomousStarted = false;

        //public Vision.ACTIVE_WEBCAM activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
        public GameField.VISION_IDENTIFIED_TARGET targetZone = GameField.VISION_IDENTIFIED_TARGET.LEVEL1;

        double af = GameField.ALLIANCE_FACTOR;

        Trajectory traj;


        @Override
        public void runOpMode() throws InterruptedException {
            /* Create Subsystem Objects*/
            driveTrain = new org.firstinspires.ftc.teamcode.Subsystems.DriveTrain(hardwareMap);
            SSElevator = new SSElevator(hardwareMap, this);
            SSArm = new SSArm(hardwareMap);


            gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, SSClaw, SSElevator, SSArm, this);
            autonomousController = new AutonomousController(driveTrain, SSElevator, SSClaw, SSArm);

            GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
            //Key Pay inputs to select Game Plan;
              //vision = new Vision(hardwareMap, activeWebcam);
            af = GameField.ALLIANCE_FACTOR;

            // Initiate Camera on Init.
            //  vision.activateVuforiaTensorFlow();

            // 1.  Robot starts position with arm at Level Intake Front with pre-loaded cone in claw.
            startPose = RED_WAREHOUSE_STARTPOS;
            driveTrain.getLocalizer().setPoseEstimate(startPose);

            //Robot starts with Elevator in Collect State, with preloaded box
            //On Init, Elevator moves to Level 1, Bucket moves to transport

            autonomousController.runAutoControl();

            telemetry.addData("Start Pose : ", "Red Right Forward");
            telemetry.addData("Waiting for start to be pressed.", "Robot is ready!");
            telemetry.update();

            if (isStopRequested()) return;

            while (!isStopRequested()) {
                // 2.  Call Vision function to detect Team Element position and set parking position
                //Run Vuforia Tensor Flow
                //  targetZone = vision.runVuforiaTensorFlow();

                if (!parked) {
                    autonomousController.runAutoControl();
                }

                if (DEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

                //Game Play is pressed
                while (opModeIsActive() && !isStopRequested() && !parked) {

                    //  vision.deactivateVuforiaTensorFlow();
                    runAutoRedRightForward();

                    //Move to Launching Position
                    parked = true;

                    //Write last position to static class to be used as initial position in TeleOp
                    GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
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
            GameField.poseSetInAutonomous = true;            autonomousController.autoSSArmSetToIntakeForward();
            autonomousController.autoSSElevatorSetToLevelLow();
            autonomousController.autoSSClawSetToClose();
            autonomousController.runAutoControl();

            //Logic for waiting
            safeWait(1000);

            // 3.  Call roadrunner function to move to high junction 4
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(12, 30, Math.toRadians(225)))
                    .build();

            driveTrain.followTrajectory(traj);

            // 4.  Call arm function to move to arm level high
            autonomousController.autoSSArmSetToHigh();
            safeWait(1000);

            // 5.  Call claw function to open claw
            autonomousController.autoSSClawSetToOpen();

            //6
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(14, 56, Math.toRadians(90)))
                    .build();

            driveTrain.followTrajectory(traj);


            //7. Call arm to pick up cone, claw to close, arm to carrying position
            autonomousController.autoSSArmSetToIntakeForward();
            autonomousController.autoSSClawSetToClose();


            // 8.  Call roadrunner function to move to high junction 4
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(12, 30, Math.toRadians(225)))
                    .build();

            driveTrain.followTrajectory(traj);


            // 9.  Call arm function to move to arm level high
            autonomousController.autoSSArmSetToHigh();
            safeWait(1000);

            // 10.  Call claw function to open claw
            autonomousController.autoSSClawSetToOpen();

            // 11. Pick up a cone to store
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(14, 56, Math.toRadians(90)))
                    .build();

            driveTrain.followTrajectory(traj);


            // 12. Call arm to pick up cone, claw to close, arm to carrying position
            autonomousController.autoSSArmSetToIntakeForward();
            autonomousController.autoSSClawSetToClose();



            // 13. Call roadrunner function to park in detected position
            switch (targetZone) {
                case LEVEL1:
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(36 , 60, Math.toRadians(180)))
                            .build();
                    break;
                case LEVEL2:
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(180)))
                            .build();
                    break;
                case LEVEL3:
                case UNKNOWN:
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(180)))
                            .build();
                    break;
            }
            autonomousController.autoSSArmSetToIntakeForward();
            safeWait(1000);

        }

        public void runAutoRedRightForward() {
        }

        /**
         * Safe method to wait so that stop button is also not missed
         *
         * @param time time in ms to wait
         */

        public void safeWait(double time) {
            ElapsedTime timer = new ElapsedTime(MILLISECONDS);
            timer.reset();
            while (!isStopRequested() && timer.time() < time) {
                autonomousController.runAutoControl();
            }
        }


        /*       public void dropFreightLevel1(){
                   autonomousController.autoSSElevatorSetToLevel1();
                   safeWait(1000);
                   autonomousController.autoBucketSetToCollect();
                   safeWait(1000);
               }

               public void moveElevatorToLevel1(){
                   autonomousController.autoSSElevatorSetToLevel1();
               }
               public void moveElevatorToLevel0(){
                   autonomousController.autoSSElevatorSetToLevel0();
               }


                // Hybrid Commands For Autonomous OpMode


               /**
                * Method to add debug messages. Update as telemetry.addData.
                * Use public attributes or methods if needs to be called here.
                */
        public void printDebugMessages() {
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
/*          telemetry.addData("Target Label : ", vision.detectedLabel);
            telemetry.addData("Target Left : ", vision.detectedLabelLeft);
            telemetry.addData("Target Right : ", vision.detectedLabelRight);
            telemetry.addData("Target Top : ", vision.detectedLabelTop);
            telemetry.addData("Target Bottom : ", vision.detectedLabelBottom);
            telemetry.addData("Vision targetLevelDetected : ", vision.targetLevelDetected);
            telemetry.addData("Vision detectedLabel", vision.detectedLabel);
            telemetry.addData("Vision detectedLabelLeft :", vision.detectedLabelLeft);
            */
            telemetry.addData("Elevator level value : ", SSElevator.getElevatorPosition());
            telemetry.addData("Elevator Encoder Current :", SSElevator.elevatorMotorLeft.getCurrentPosition());
            telemetry.addData("Elevator Encoder Current :", SSElevator.elevatorMotorRight.getCurrentPosition());
            telemetry.addData("Elevator Encoder Target:", SSElevator.elevatorMotorLeft.getTargetPosition());
            telemetry.addData("Elevator Encoder Target:", SSElevator.elevatorMotorRight.getTargetPosition());
            telemetry.addData("Arm Position : ", SSArm.getArmPosition());
            telemetry.addData("Arm Grip State : ", SSClaw.getGripServoState());

            telemetry.update();


        }
    }
}