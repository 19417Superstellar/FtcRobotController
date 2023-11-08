package org.firstinspires.ftc.teamcode.TestOpModes;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSClaw;
import org.firstinspires.ftc.teamcode.SubSystems.SSElevator;

import java.util.Observer;
public class TeleOp_Elevator {
    public boolean DEBUG_FLAG = true;

    public GamepadController gamepadController;
    public DriveTrain driveTrain;

    //TODO_SS
    public SSElevator ssElevator;
    public SSClaw ssClaw;

    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;
    private boolean stopRequested;

    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.SUPERSTELLAR_TELEOP;
        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap, new Pose2d(0,0,0), telemetry);

        //TODO_SS
        ssElevator = new SSElevator(hardwareMap, (Telemetry) this);
        ssClaw = new SSClaw(hardwareMap,telemetry);

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1,gamepad2,driveTrain,ssClaw,ssElevator);

        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if(DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (isStopRequested()) {
                gamepadController.runSSElevator();

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
        telemetry.addData("Drive Mode :", driveTrain.driveType);
        telemetry.addData("PoseEstimate :", driveTrain.driveType);
        telemetry.addData("elevator_motor_encoder_left", ssElevator.currentLeftEncoderValue());
        telemetry.addData("elevator_motor_encoder_right",ssElevator.currentRightEncoderValue());

        //add for all subsytems

        telemetry.update();
    }

    public boolean isStopRequested() {
        return stopRequested;
    }
}
