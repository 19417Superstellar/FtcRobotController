package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SSClaw;
import org.firstinspires.ftc.teamcode.SubSystems.SSElevator;
import org.firstinspires.ftc.teamcode.SubSystems.SSHoldingPen;
import org.firstinspires.ftc.teamcode.SubSystems.SSIndicators;
import org.firstinspires.ftc.teamcode.SubSystems.SSIntake;
import org.firstinspires.ftc.teamcode.SubSystems.SSRocketLauncher;

@TeleOp(name = "indicator teleop", group = "Test")
public class TeleOp_Indicators extends LinearOpMode {
    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public SSIndicators indicators;
    public Pose2d startPose = GameField.ORIGINPOSE;
    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);

    /*
     * Constructor for passing all the subsystems in order to make the subsystem be able to use
     * and work/be active
     */

    public void runOpMode() throws InterruptedException {
        indicators = new SSIndicators(hardwareMap, telemetry);
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, null, null, null,
                null, indicators, null, telemetry, this);


        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.SUPERSTELLAR_TELEOP;

        /* Set Initial State of any subsystem when OpMode is to be started*/
        indicators.initIndicators();

        /* Wait for Start or Stop Button to be pressed */
        gameTimer.reset();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                indicators.printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                gamepadController.runSSIndicators();
                indicators.printDebugMessages();
                telemetry.update();

                if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                    indicators.printDebugMessages();
                    telemetry.update();
                }
            }
        }
        GameField.poseSetInAutonomous = false;
    }
}
