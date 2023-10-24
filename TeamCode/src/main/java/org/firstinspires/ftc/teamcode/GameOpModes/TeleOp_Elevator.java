package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.SSClaw;
import org.firstinspires.ftc.teamcode.Subsystems.SSElevator;
import org.firstinspires.ftc.teamcode.Subsystems.SSHoldingPen;

@TeleOp(name = "TeleOp_Elevator", group = "00-TeleOp")
public class TeleOp_Elevator extends LinearOpMode{

        public boolean DEBUG_FLAG = true;

        public GamepadController gamepadController;
        public DriveTrain driveTrain;

        //TODO_SS
        public SSElevator ssElevator;
        public SSClaw ssClaw;

        //public Vuforia Vuforia1;
        public Pose2d startPose = GameField.ORIGINPOSE;
    private SSHoldingPen SSElevator;

    @Override
        public void runOpMode() throws InterruptedException {

            /* Create Subsystem Objects*/
            driveTrain = new DriveTrain(hardwareMap);

            //TODO_SS
            ssElevator = new SSElevator(hardwareMap, this);
            ssClaw = new SSClaw(hardwareMap);

            /* Create Controllers */
            LinearOpMode SSClaw = null;
            gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, SSClaw, SSElevator, this);
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
                    gamepadController.runTemp();

                    if(DEBUG_FLAG) {
                        printDebugMessages();
                        telemetry.update();
                    }

                }

            }
            //GameField.poseSetInAutonomous = false;
        }

        /**
         * Method to add debug messages. Update as telemetry.addData.
         * Use public attributes or methods if needs to be called here.
         */
        public void printDebugMessages(){
            telemetry.setAutoClear(true);
            telemetry.addData("DEBUG_FLAG is : ", DEBUG_FLAG);

            telemetry.addData("GameField.playingAlliance : ", GameField.playingAlliance);
            telemetry.addData("startPose :", startPose);

            telemetry.addData("Drive Mode :", driveTrain.driveMode);
            telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
            //telemetry.addData("Battery Power :", driveTrain.getBatteryVoltage(hardwareMap));
            telemetry.addData("elevator_motor_encoder_left", ssElevator.currentLeftEncoderValue());
            telemetry.addData("elevator_motor_encoder_right",ssElevator.currentRightEncoderValue());

            telemetry.update();
        }
}
