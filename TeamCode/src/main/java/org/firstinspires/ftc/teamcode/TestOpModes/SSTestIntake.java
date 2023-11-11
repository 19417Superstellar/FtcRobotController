package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSIntake;


@TeleOp(name = "SS Test Intake", group = "Test")
public class SSTestIntake extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public SSIntake ssIntake;
    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.SUPERSTELLAR_TELEOP;

        ssIntake = new SSIntake(hardwareMap, telemetry);
        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, ssIntake, null, null,
                null, null, null, telemetry, this);

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        ssIntake.initSSIntake();

        gameTimer.reset();

        /* Wait for Start or Stop Button to be pressed */
        waitForStart();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /* Wait for Start or Stop Button to be pressed */
        /* If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                ssIntake.printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                gamepadController.runSSIntake();
                ssIntake.printDebugMessages();
                telemetry.update();

                if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                    ssIntake.printDebugMessages();
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
        telemetry.update();

        ssIntake.printDebugMessages();
    }
}



