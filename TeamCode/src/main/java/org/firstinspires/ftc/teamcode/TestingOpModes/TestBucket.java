package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSBucket;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Superstellar Robot for Freight Frenzy.<BR>
 *
 */
@TeleOp(name = "TestBucket", group = "Test")
@Disabled
public class TestBucket extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadTestController gamepadTestController;
    public DriveTrain driveTrain;
    public SSBucket ssBucket;

    boolean gp1ButtonALast = false;
    boolean gp1ButtonBLast = false;
    boolean gp1ButtonXLast = false;
    boolean gp1ButtonYLast = false;
    boolean gp1RightBumperLast = false;

    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        //Declare subsystem to be tested
        ssBucket = new SSBucket(hardwareMap);
        /* Create Controllers */
        gamepadTestController = new GamepadTestController(gamepad1, driveTrain);

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        ssBucket.initBucket();

        /* Wait for Start or Stop Button to be pressed */
        waitForStart();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if (DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                gamepadTestController.runByGamepadControl();

                //Test Code
                if (gamepadTestController.getButtonAPress()) {
                    if (ssBucket.getBucketServoState() != SSBucket.BUCKET_SERVO_STATE.COLLECT_POSITION) {
                        ssBucket.setToCollect();
                    }

                }
                if (gamepadTestController.getButtonXPress()) {
                    if (ssBucket.getBucketServoState() != SSBucket.BUCKET_SERVO_STATE.TRANSPORT_POSITION) {
                        ssBucket.setToTransport();
                    }
                }
                if (gamepadTestController.getButtonYPress()) {
                    if (ssBucket.getBucketServoState() != SSBucket.BUCKET_SERVO_STATE.TRANSPORT_POSITION) {
                        ssBucket.setToTransport();
                    }
                }
                if (gamepadTestController.getButtonBPress()) {
                    if (ssBucket.getBucketServoState() != SSBucket.BUCKET_SERVO_STATE.TRANSPORT_POSITION) {
                        ssBucket.setToTransport();
                    }
                }
                if (gamepadTestController.getRightBumperPress()) {
                    if (ssBucket.getBucketServoState() != SSBucket.BUCKET_SERVO_STATE.DROP_POSITION) {
                        ssBucket.setToDrop();
                    } else if (ssBucket.getBucketServoState() == SSBucket.BUCKET_SERVO_STATE.DROP_POSITION) {
                        ssBucket.setToTransport();
                    }
                }
            }

            if (DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
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

        telemetry.addData("Subsystem1 State : ", ssBucket.getBucketServoState());

        //Add logic for debug print Logic

        telemetry.update();

    }
}
