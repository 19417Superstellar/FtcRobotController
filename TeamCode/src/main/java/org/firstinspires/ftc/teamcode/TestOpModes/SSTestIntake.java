package org.firstinspires.ftc.teamcode.TestOpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSIntake;


    @TeleOp(name = "Test Intake", group = "Test")
    public class SSTestIntake extends LinearOpMode {

        public boolean DEBUG_FLAG = true;

        public GamepadController gamepadController;
        public DriveTrain driveTrain;
        public SSIntake ssIntake;

        //public Vuforia Vuforia1;
        public Pose2d startPose = GameField.ORIGINPOSE;

        @Override
        public void runOpMode() throws InterruptedException {

            /* Create Subsystem Objects*/
            driveTrain = new DriveTrain(hardwareMap, new Pose2d(0,0,0), telemetry);
            //Declare subsystem to be tested
            ssIntake = new SSIntake(hardwareMap, telemetry);
            /* Create Controllers */

            gamepadController = new GamepadController(gamepad1,gamepad2,driveTrain);

            /* Set Initial State of any subsystem when TeleOp is to be started*/
            ssIntake.initSSIntake();

            /* Wait for Start or Stop Button to be pressed */
            waitForStart();

            /* If Stop is pressed, exit OpMode */
            if (isStopRequested()) return;

            /*If Start is pressed, enter loop and exit only when Stop is pressed */
            while (!isStopRequested()) {

                if(DEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

                while (opModeIsActive()) {
                    gamepadController.runByGamepadControl();

                    //Test Code
                    if (gamepadController.gp1GetDpad_downPress()) {
                        if(ssIntake.getSSIntakeMotorState() != SSIntake.SSINTAKE_MOTOR_STATE.RUNNING) {
                            ssIntake.startForwardSSIntakeMotor();
                        } else if(ssIntake.getSSIntakeMotorState() != SSIntake.SSINTAKE_MOTOR_STATE.STOPPED) {
                            ssIntake.stopSSIntakeMotor();
                        }
                    }

                    //Reverse Intake motors and run - in case of stuck state)
                    if (gamepadController.gp1GetDpad_upPress()) {
                        if(ssIntake.getSSIntakeMotorState() != SSIntake.SSINTAKE_MOTOR_STATE.REVERSING) {
                            ssIntake.startReverseSSIntakeMotor();
                        } else if (ssIntake.getSSIntakeMotorState() != SSIntake.SSINTAKE_MOTOR_STATE.STOPPED) {
                            ssIntake.stopSSIntakeMotor();
                        }
                    }

                    if(DEBUG_FLAG) {
                        printDebugMessages();
                        telemetry.update();
                    }

                }

            }
            GameField.poseSetInAutonomous = false;
        }

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
            telemetry.update();
        }

            //****** Drive debug ******
           /*telemetry.addData("Drive Mode : ", driveTrain.driveMode);
            telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
            telemetry.addData("Battery Power : ", driveTrain.getBatteryVoltage(hardwareMap));

            telemetry.addData("Intake State : ", ssIntake.getSSIntakeMotorState()); */

            //Add logic for debug print Logic



        }



