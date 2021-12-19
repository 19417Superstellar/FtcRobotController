package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSElevator;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "Test Elevator", group = "Test")
@Disabled
public class TestElevator extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadTestController gamepadTestController;
    public DriveTrain driveTrain;
    public SSElevator ssElevator;

    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        //Declare subsystem to be tested
        ssElevator = new SSElevator(hardwareMap);
        /* Create Controllers */
        gamepadTestController = new GamepadTestController(gamepad1, driveTrain);

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        ssElevator.initElevator();


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

                //Test Code

                ///START_BUTTON_CHECK: Checking button action and setting the elevator params to the required level
                if (gamepadTestController.getButtonAPress()) {
                    if(ssElevator.getElevatorPosition() != SSElevator.ELEVATOR_POSITION.LEVEL_0) {
                        ssElevator.moveElevatorLevel0();
                    }

                }

                if (gamepadTestController.getButtonXPress()) {
                    if(ssElevator.getElevatorPosition() != SSElevator.ELEVATOR_POSITION.LEVEL_1) {
                        ssElevator.moveElevatorLevel1();
                    }
                }

                if (gamepadTestController.getButtonBPress()) {
                    if(ssElevator.getElevatorPosition() != SSElevator.ELEVATOR_POSITION.LEVEL_2) {
                        ssElevator.moveElevatorLevel2();
                    }
                }

                if (gamepadTestController.getButtonYPress()) {
                    if(ssElevator.getElevatorPosition() != SSElevator.ELEVATOR_POSITION.LEVEL_3) {
                        ssElevator.moveElevatorLevel3();
                    }
                }

                ///END_BUTTON_CHECK:

                //calling the elevator move function for the motor
                if (ssElevator.runElevatorToLevelState) {
                    ssElevator.runElevatorToLevel(ssElevator.motorPowerToRun);
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

        telemetry.addData("Elevator level value : ", ssElevator.getElevatorPosition());

        //Add logic for debug print Logic

        telemetry.update();

    }
}
