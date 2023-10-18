package org.firstinspires.ftc.teamcode.GameOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.SSHoldingPen;

@TeleOp(name = "TeleOp_HoldingPenTest", group = "00-TeleOp")
public class AbbyMode extends LinearOpMode {

    public GamepadController gamepadController;

    public SSHoldingPen holdingPen;

    public DriveTrain driveTrain;

    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);

        holdingPen = new SSHoldingPen(hardwareMap, this);

        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, this, holdingPen);
        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()){
            gamepadController.runTemp();
        }
    }
}
