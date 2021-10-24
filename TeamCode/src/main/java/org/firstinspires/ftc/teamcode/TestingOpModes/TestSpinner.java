package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSSpinner;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "SpinnerTest", group = "Test")
public class TestSpinner extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadTestController gamepadTestController;
    public DriveTrain driveTrain;
    public SSSpinner ssSpinner;

    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        //TODO: Declare subsystem to be tested
        ssSpinner = new SSSpinner(hardwareMap);
        /* Create Controllers */
        gamepadTestController = new GamepadTestController(gamepad1, driveTrain);

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        //ssSpinner.init();

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
                gamepadTestController.runByGamepadControl();

                //TODO: Add Test Code here
                /* lOGIC AS FOLLOWS:
                When GameField.playingAlliance is GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE {
                 if gamepadTestController.getLeftBumperPress(), then
                  if spinner is stopped, start motor anticlockwise
                  else if spinner is not stopped, stop motor
                 if gamepadTestController.getLeftBumperPress() and startup button is pressed - gamepadTestController.getStartPersistent(), then
                  if spinner is stopped, start motor clockwise
                  else if spinner is not stopped, stop motor
                }
                Reverse the clockwise and anticlockwise logic for RED ALLiance
                 */

                //* TODO : Add logic for checking alliance color
                if (gamepadTestController.getLeftBumperPress()) {
                    if(ssSpinner.getSSSpinnerMotorState() == SSSpinner.SSSPINNER_MOTOR_STATE.STOPPED) {
                        ssSpinner.startClockwiseSSSpinnerMotor();
                    } else if(ssSpinner.getSSSpinnerMotorState()!= SSSpinner.SSSPINNER_MOTOR_STATE.CLOCKWISE) {
                        ssSpinner.stopSSSpinnerMotor();
                    }
                }

                //Reverse Intake motors and run - in case of stuck state)
                if (gamepadTestController.getLeftBumperPersistant() && gamepadTestController.getStartPersistent()) {
                    if(ssSpinner.getSSSpinnerMotorState() !=SSSpinner.SSSPINNER_MOTOR_STATE.ANTICLOCKWISE) {
                        ssSpinner.startAntiClockwiseSSSpinnerMotor();
                    } else if(ssSpinner.getSSSpinnerMotorState()!= SSSpinner.SSSPINNER_MOTOR_STATE.STOPPED) {
                        ssSpinner.stopSSSpinnerMotor();
                    }
                }

                //TODO: Add code for other alliance color

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
        telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
        telemetry.addData("GameField.currentPose : ", GameField.currentPose);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", driveTrain.driveMode);
        telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
        telemetry.addData("Battery Power : ", driveTrain.getBatteryVoltage(hardwareMap));

        telemetry.addData("SSSpinner State : ", ssSpinner.getSSSpinnerMotorState() );

        //Add logic for debug print Logic

        telemetry.update();

    }
}