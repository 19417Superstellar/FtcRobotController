package org.firstinspires.ftc.teamcode.GameOpModes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.SSArm;
import org.firstinspires.ftc.teamcode.Subsystems.SSClaw;
import org.firstinspires.ftc.teamcode.Subsystems.SSElevator;
import org.firstinspires.ftc.teamcode.Subsystems.SSHoldingPen;
import org.firstinspires.ftc.teamcode.Subsystems.SSIndicators;
import org.firstinspires.ftc.teamcode.Subsystems.SSIntake;

/**
 * Power Play TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by SuperStellar Robot for Freight Frenzy.<BR>
 *
 */

@TeleOp(name = "TeleOp_Claw", group = "00-Teleop")
public class TeleOp_Claw extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadController gamepadController;
    public DriveTrain driveTrain;

    //TODO_SS
    public SSClaw ssClaw;
    public SSElevator ssElevator;
    public SSIndicators ssIndicators;
    public SSHoldingPen ssHoldingPen;
    public SSIntake ssIntake;


    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);

        //TODO_SS
        ssClaw = new SSClaw(hardwareMap);
        ssElevator = new SSElevator(hardwareMap, this);
        ssIndicators = new SSIndicators(hardwareMap, this);
        ssHoldingPen = new SSHoldingPen(hardwareMap, this);
        ssIntake = new SSIntake(hardwareMap, this);

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, ssClaw, ssElevator, ssHoldingPen, ssIndicators, ssIntake, this);


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

        double servoPosition = 0.1;
        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if(DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                gamepadController.runClaw();
                if (gamepadController.gp1GetButtonXPress()) {
                    if (ssClaw.getGripServoState() == SSClaw.GRIP_SERVO_STATE.GRIP_OPEN) {
                        ssClaw.setGripClose();
                        ssClaw.gripServo.setPosition(SSClaw.GRIP_OPEN_POSITION);
                        ssClaw.gripServoState = SSClaw.GRIP_SERVO_STATE.GRIP_OPEN;
                    } else {
                        ssClaw.setGripOpen();
                        ssClaw.gripServo.setPosition(SSClaw.GRIP_CLOSE_POSITION);
                        ssClaw.gripServo.setPosition(SSClaw.GRIP_CLOSE_POSITION);
                        ssClaw.gripServoState = SSClaw.GRIP_SERVO_STATE.GRIP_CLOSE;
                    }
                }

                if (gamepadController.gp1GetDpad_downPress()){
                    if (servoPosition >0) servoPosition -=0.01;
                }
                if (gamepadController.gp1GetDpad_upPress()) {
                    if (servoPosition <1.0) servoPosition +=0.01;
                }

                ssClaw.gripServo.setPosition(servoPosition);



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
        //telemetry.addData("GameField.poseSetInAutonomous :", GameField.poseSetInAutonomous);
        ///telemetry.addData("GameField.currentPose :", GameField.currentPose);
        telemetry.addData("startPose :", startPose);

        telemetry.addData("Drive Mode :", driveTrain.driveMode);
        telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
        //telemetry.addData("Battery Power :", driveTrain.getBatteryVoltage(hardwareMap));

        //add for all subsytems
        telemetry.addData("grip_position",ssClaw.getGripServoState());
        telemetry.addData("grip_position_value_left",ssClaw.gripServo.getPosition());
        telemetry.addData("wrist_position_value_left",ssClaw.wristServo.getPosition());


        telemetry.update();
    }
}
