package org.firstinspires.ftc.teamcode.GameOpModes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.SSArm;
import org.firstinspires.ftc.teamcode.Subsystems.SSClaw;
import org.firstinspires.ftc.teamcode.Subsystems.SSElevator;

/**
 * Power Play TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by SuperStellar Robot for Freight Frenzy.<BR>
 *
 */

@TeleOp(name = "TeleOp_Arm", group = "00-Teleop")
public class TeleOp_Arm extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadController gamepadController;
    public DriveTrain driveTrain;

    //TODO_SS
    public SSArm ssArm;
    public SSElevator ssElevator;
    public SSClaw ssClaw;

    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);

        //TODO_SS
        ssArm = new SSArm(hardwareMap);
        ssClaw = new SSClaw(hardwareMap);
        ssElevator = new SSElevator(hardwareMap, this);

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, ssClaw, ssElevator, ssArm);


        //Get last position after Autonomous mode ended from static class set in Autonomous
        if (GameField.poseSetInAutonomous) {
            driveTrain.getLocalizer().setPoseEstimate(GameField.currentPose);
        } else {
                driveTrain.getLocalizer().setPoseEstimate(startPose);
        }

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
                gamepadController.runArm();

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
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_FLAG is : ", DEBUG_FLAG);

        telemetry.addData("GameField.playingAlliance : ", GameField.playingAlliance);
        telemetry.addData("GameField.poseSetInAutonomous :", GameField.poseSetInAutonomous);
        telemetry.addData("GameField.currentPose :", GameField.currentPose);
        telemetry.addData("startPose :", startPose);

        telemetry.addData("Drive Mode :", driveTrain.driveMode);
        telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
        telemetry.addData("Battery Power :", driveTrain.getBatteryVoltage(hardwareMap));
        telemetry.addData("arm_motor_encoder_left",ssArm.currentEncoderValueLeft());
        telemetry.addData("arm_motor_encoder_right",ssArm.currentEncoderValueRight());

        //add for all subsytems

        telemetry.update();
    }
}
