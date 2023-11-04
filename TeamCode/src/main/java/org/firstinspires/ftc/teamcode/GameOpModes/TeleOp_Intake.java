package org.firstinspires.ftc.teamcode.GameOpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;

public class TeleOp_Intake {
    DcMotor intakeMotor;
// You may have other hardware components like servos, sensors, etc.

    public void init() {
        intakeMotor = hardwareMap.dcMotor.get("intake_motor");
        // Initialize and configure other hardware components here
    }
    public void loop() {
        // Read gamepad inputs
        double intakePower = 0.0;

        // You can use different gamepad buttons or triggers for control
        if (GamepadController.a) {
            intakePower = 0.5; // Spin the intake motor forward
        } else if (GamepadController.b) {
            intakePower = -0.5; // Reverse the intake motor
        } else {
            intakePower = 0.0; // Stop the intake motor
        }

        // Set the intake motor power
        intakeMotor.setPower(intakePower);

        // Add other teleop control logic for different components

        //...
    }


}
