package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSBucket;
import org.firstinspires.ftc.teamcode.SubSystems.SSHoldingPen;

@TeleOp(name = "SS Test Holding Pen", group = "Test")
public class SSTestHoldingPen extends LinearOpMode {
    public boolean DEBUG_FLAG = true;
    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public SSHoldingPen ssHoldingPen;
    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);

    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.SUPERSTELLAR_TELEOP;

        ssHoldingPen = new SSHoldingPen(hardwareMap, telemetry);

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, null, null, ssHoldingPen,null,
                null, null, telemetry, this);

        ssHoldingPen.initSSHoldingPen();

        gameTimer.reset();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /* Wait for Start or Stop Button to be pressed */
        /* If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                ssHoldingPen.printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                gamepadController.runSSHoldingPen();
                ssHoldingPen.printDebugMessages();
                telemetry.update();

                if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                    ssHoldingPen.printDebugMessages();
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


        //add for all subsytems
        ssHoldingPen.printDebugMessages();

        telemetry.update();
    }

}

