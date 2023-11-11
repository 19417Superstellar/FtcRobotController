package org.firstinspires.ftc.teamcode.TestOpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSElevator;

// [AG] - Keethika - we need the TeleOp attribute
@TeleOp(name = "SS Test Elevator", group = "Test")
public class SSTestElevator extends LinearOpMode {
    public boolean DEBUG_FLAG = true;

    public GamepadController gamepadController;
    public DriveTrain driveTrain;

    //TODO_SS
    public SSElevator ssElevator;

    public Pose2d startPose = GameField.ORIGINPOSE;

    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.SUPERSTELLAR_TELEOP;

        //TODO_SS
        ssElevator = new SSElevator(hardwareMap, telemetry);

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, null, ssElevator, null,
                null, null, null, telemetry, this);

        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            gamepadController.runSSElevator();

            if(DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (isStopRequested()) {

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

        //add for all subsytems
        ssElevator.printDebugMessages();

        telemetry.update();
    }

}
